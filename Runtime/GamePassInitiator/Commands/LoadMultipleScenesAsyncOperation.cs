using Bloodthirst.Core.SceneManager;
using Bloodthirst.Core.Utils;
using Bloodthirst.Scripts.Core.GamePassInitiator;
using Bloodthirst.System.CommandSystem;
using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.Pool;
using UnityEngine.SceneManagement;

namespace Bloodthirst.Core.Setup
{
    public class LoadMultipleScenesAsyncWrapper : IAsynOperationWrapper
    {
        private readonly IList<string> scenes;
        private readonly bool triggerCallbacks;
        public LoadMultipleScenesAsyncWrapper(IList<string> scenes, bool triggerCallbacks)
        {
            this.scenes = scenes;
            this.triggerCallbacks = triggerCallbacks;
        }

        public bool ShouldExecute()
        {
            foreach (string s in scenes)
            {
                Scene scene = UnityEngine.SceneManagement.SceneManager.GetSceneByPath(s);

                if (!scene.isLoaded)
                    return true;
            }

            return false;
        }

        IProgressCommand IAsynOperationWrapper.CreateOperation()
        {
            return new LoadMultipleScenesAsyncOperation(scenes, triggerCallbacks);
        }
    }

    public class LoadMultipleScenesAsyncOperation : TreeCommandBase<LoadMultipleScenesAsyncOperation>, IProgressCommand
    {
        private readonly IList<string> scenes;
        private readonly bool triggerCallbacks;
        private List<LoadSingleSceneAsyncOperation> subCommands;
        public LoadMultipleScenesAsyncOperation(IList<string> scenes, bool triggerCallbacks) : base()
        {
            this.scenes = scenes;
            this.triggerCallbacks = triggerCallbacks;
            subCommands = new List<LoadSingleSceneAsyncOperation>();
        }

        private float currentProgress;
        public float CurrentProgress
        {
            get => currentProgress;
            private set
            {
                if (currentProgress == value)
                    return;

                float old = currentProgress;
                currentProgress = value;

                OnCurrentProgressChanged?.Invoke(this, old, currentProgress);
            }
        }

        public event Action<IProgressCommand, float, float> OnCurrentProgressChanged;
        string IProgressCommand.TaskName => $"Loading {scenes.Count} scenes";
        public override void OnStart()
        {
            for (int i = 0; i < scenes.Count; i++)
            {
                string scene = scenes[i];

                LoadSingleSceneAsyncOperation cmd = new LoadSingleSceneAsyncOperation(scene, false);

                cmd.OnCurrentProgressChanged += Cmd_OnCurrentProgressChanged;

                subCommands.Add(cmd);

                Append(cmd);
            }

            base.OnStart();
        }

        private void Cmd_OnCurrentProgressChanged(IProgressCommand arg1, float arg2, float arg3)
        {
            Refresh();
        }

        private void Refresh()
        {
            float curr = 0;
            float total = subCommands.Count;

            for (int i = 0; i < total; i++)
            {
                LoadSingleSceneAsyncOperation op = subCommands[i];

                curr += op.CurrentProgress;
            }

            CurrentProgress = curr / total;


            if (currentProgress != 1)
                return;

            // scenes are done loading

            for (int i = 0; i < total; i++)
            {
                LoadSingleSceneAsyncOperation op = subCommands[i];

                op.OnCurrentProgressChanged -= Cmd_OnCurrentProgressChanged;
            }

            if (triggerCallbacks)
            {
                using (ListPool<GameObject>.Get(out List<GameObject> sceneGOs))
                using (ListPool<IOnSceneLoaded>.Get(out List<IOnSceneLoaded> loaded))
                using (ListPool<ISceneInstanceManager>.Get(out List<ISceneInstanceManager> manager))
                {
                    for (int i = 0; i < scenes.Count; i++)
                    {
                        string scenePath = scenes[i];

                        Scene scene = UnityEngine.SceneManagement.SceneManager.GetSceneByPath(scenePath);

                        // note : this does a clear already so no need to do it ourselves
                        scene.GetRootGameObjects(sceneGOs);

                        // trigger scene callbacks
                        GameObjectUtils.GetAllComponents(ref loaded, sceneGOs, true);
                        GameObjectUtils.GetAllComponents(ref manager, sceneGOs, true);

                        Assert.IsTrue(manager.Count == 1);

                        ISceneInstanceManager sceneInstance = manager[0];

                        for (int j = 0; j < loaded.Count; j++)
                        {
                            IOnSceneLoaded e = loaded[j];
                            e.OnLoaded(sceneInstance);
                        }
                    }
                }
            }

            subCommands.Clear();

            Success();
        }
    }
}
