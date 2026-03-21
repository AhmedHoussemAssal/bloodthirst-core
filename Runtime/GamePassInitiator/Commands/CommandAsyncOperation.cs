using Bloodthirst.Core.SceneManager;
using Bloodthirst.System.CommandSystem;
using System;
using System.Collections.Generic;

namespace Bloodthirst.Core.Setup
{
    public class CommandAsyncWrapper : IAsynOperationWrapper
    {
        private readonly ICommandBase cmd;

        public CommandAsyncWrapper(ICommandBase cmd, bool removeWhenDone)
        {
            this.cmd = cmd;
            cmd.RemoveWhenDone = removeWhenDone;
        }

        IProgressCommand IAsynOperationWrapper.CreateOperation()
        {
            return new CommandAsyncOperation(cmd);
        }
    }

    public class CommandAsyncOperation : CommandBase<CommandAsyncOperation> , IProgressCommand
    {
        public CommandAsyncOperation(ICommandBase cmd)
        {
            this.cmd = cmd;
        }

        private ICommandBase cmd;
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
        string IProgressCommand.TaskName => $"Executing command {cmd.GetType().Name}";
        public override void OnStart()
        {
            cmd.Start();
        }

        public override void OnTick(float delta)
        {
            cmd.OnTick(delta);

            if (cmd.IsDone())
            {
                CurrentProgress = 1;
                Success();
            }
        }
    }
}
