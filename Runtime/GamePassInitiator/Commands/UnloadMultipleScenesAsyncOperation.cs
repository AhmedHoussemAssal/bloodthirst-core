using Bloodthirst.Core.SceneManager;
using Bloodthirst.System.CommandSystem;
using System;
using System.Collections.Generic;

namespace Bloodthirst.Core.Setup
{
    public class UnloadMultipleScenesAsyncWrapper : IAsynOperationWrapper
    {
        private readonly IList<ISceneInstanceManager> scenes;

        public UnloadMultipleScenesAsyncWrapper(IList<ISceneInstanceManager> scenes)
        {
            this.scenes = scenes;
        }

        IProgressCommand IAsynOperationWrapper.CreateOperation()
        {
            return new UnloadMultipleScenesAsyncOperation(scenes);
        }
    }

    public class UnloadMultipleScenesAsyncOperation : TreeCommandBase<LoadMultipleScenesAsyncOperation>, IProgressCommand
    {
        private readonly IList<ISceneInstanceManager> scenes;
        private List<UnloadSingleSceneAsyncOperation> subCommands;
        public UnloadMultipleScenesAsyncOperation(IList<ISceneInstanceManager> scenes) : base()
        {
            this.scenes = scenes;

            subCommands = new List<UnloadSingleSceneAsyncOperation>();
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
                ISceneInstanceManager scene = scenes[i];

                UnloadSingleSceneAsyncOperation cmd = new UnloadSingleSceneAsyncOperation(scene, false);

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
                UnloadSingleSceneAsyncOperation op = subCommands[i];

                curr += op.CurrentProgress;
            }

            CurrentProgress = curr / total;


            if (currentProgress != 1)
                return;

            for (int i = 0; i < total; i++)
            {
                UnloadSingleSceneAsyncOperation op = subCommands[i];

                op.OnCurrentProgressChanged -= Cmd_OnCurrentProgressChanged;
            }

            subCommands.Clear();

            Success();
        }
    }
}
