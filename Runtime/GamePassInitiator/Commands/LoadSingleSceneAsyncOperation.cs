using Bloodthirst.Core.BProvider;
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
    public class WaitSecondsAsyncWrapper : IAsynOperationWrapper
    {
        private readonly float seconds;

        public WaitSecondsAsyncWrapper(float seconds)
        {
            this.seconds = seconds;
        }

        public IProgressCommand CreateOperation()
        {
            return new WaitSecondsCommand(seconds);
        }

        public bool ShouldExecute()
        {
            return true;
        }
    }

    public class WaitSecondsCommand : CommandBase<WaitSecondsCommand>, IProgressCommand
    {
        public string TaskName => string.Empty;

        private float currentProgress;
        private float seconds;

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

        public WaitSecondsCommand(float seconds)
        {
            this.seconds = seconds;
        }

        public override void OnStart()
        {
            if (seconds == 0f)
                Success();
        }

        public override void OnTick(float delta)
        {
            CurrentProgress = Mathf.Clamp01(currentProgress + (delta / seconds));

            if (currentProgress == 1)
            {
                Success();
            }
        }
    }

    public class LoadSingleSceneAsyncWrapper : IAsynOperationWrapper
    {
        private readonly string scenePath;
        private readonly bool triggerSceneCallbacks;
        public LoadSingleSceneAsyncWrapper(string scenePath, bool triggerSceneCallbacks)
        {
            this.scenePath = scenePath;
            this.triggerSceneCallbacks = triggerSceneCallbacks;
        }

        public bool ShouldExecute()
        {
            Scene scene = UnityEngine.SceneManagement.SceneManager.GetSceneByPath(scenePath);
            return !scene.isLoaded;
        }

        IProgressCommand IAsynOperationWrapper.CreateOperation()
        {
            return new LoadSingleSceneAsyncOperation(scenePath, triggerSceneCallbacks);
        }
    }

    public class LoadSingleSceneAsyncOperation : CommandBase<LoadSingleSceneAsyncOperation>, IProgressCommand
    {
        private readonly string scenePath;
        private readonly bool triggerSceneCallbacks;
        private AsyncOperation op;

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

        string IProgressCommand.TaskName => $"Loading Scene {scenePath}";

        public LoadSingleSceneAsyncOperation(string scene, bool triggerSceneCallbacks)
        {
            this.scenePath = scene;
            this.triggerSceneCallbacks = triggerSceneCallbacks;
        }

        public override void OnStart()
        {
            op = UnityEngine.SceneManagement.SceneManager.LoadSceneAsync(scenePath, LoadSceneMode.Additive);
        }

        public override void OnTick(float delta)
        {
            CurrentProgress = op.progress;

            if (op.isDone)
            {
                OnSceneLoadComplete();
            }
        }

        private void OnSceneLoadComplete()
        {
            Scene scene = UnityEngine.SceneManagement.SceneManager.GetSceneByPath("Assets/" + scenePath + ".unity");

            using (ListPool<GameObject>.Get(out List<GameObject> sceneGOs))
            using (ListPool<IOnSceneLoaded>.Get(out List<IOnSceneLoaded> loaded))
            using (ListPool<ISceneInstanceManager>.Get(out List<ISceneInstanceManager> manager))
            {
                scene.GetRootGameObjects(sceneGOs);

                GameObjectUtils.GetAllComponents(ref manager, sceneGOs, true);

                Assert.IsTrue(manager.Count == 1);

                ISceneInstanceManager sceneManager = manager[0];

                if (triggerSceneCallbacks)
                {
                    GameObjectUtils.GetAllComponents(ref loaded, sceneGOs, true);

                    for (int i = 0; i < loaded.Count; i++)
                    {
                        IOnSceneLoaded e = loaded[i];
                        e.OnLoaded(sceneManager);
                    }
                }

                BProviderRuntime.Instance.RegisterInstance<ISceneInstanceManager>(sceneManager);

                CurrentProgress = 1;

                Success();
            }


        }
    }
}
