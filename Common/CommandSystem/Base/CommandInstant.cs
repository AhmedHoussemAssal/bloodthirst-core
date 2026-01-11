#if ODIN_INSPECTOR
	using Sirenix.OdinInspector;
#endif
using System;
using UnityEngine;

namespace Bloodthirst.System.CommandSystem
{
    public interface ICommandInstant : ICommandBase
    {
        void Execute();
    }

    public interface ICommandInstant<TResult> : ICommandInstant
    {
        TResult GetResult();
    }


    public abstract class CommandInstant<T> : ICommandBase, ICommandInstant where T : CommandInstant<T>
    {
        public CommandDebugInfo DebugInfo { get; set; }

#if UNITY_EDITOR && ODIN_INSPECTOR
        [HideInEditorMode]
#endif
#if UNITY_EDITOR
        [SerializeField]
        private bool detailedInfo;
#endif
        public event Action<ICommandBase> OnCommandStart;

        public event Action<ICommandBase> OnCommandEnd;

        public event Action<T> OnCommandEndSpecific;

        [SerializeField]
#if UNITY_EDITOR && ODIN_INSPECTOR
        [ShowIf(nameof(detailedInfo), Value = true)]
        [HideInEditorMode]
#endif
        private COMMAND_STATE commandState;
        public COMMAND_STATE CommandState { get => commandState; set => commandState = value; }

        [SerializeField]
#if UNITY_EDITOR && ODIN_INSPECTOR
        [ShowIf(nameof(detailedInfo), Value = true)]
        [HideInEditorMode]
#endif
        private ICommandBase fallbackCommand;

        public ICommandBase FallbackCommand { get => fallbackCommand; set => fallbackCommand = value; }

        [SerializeField]
#if UNITY_EDITOR && ODIN_INSPECTOR
        [ShowIf(nameof(detailedInfo), Value = true)]
        [HideInEditorMode]
#endif
        private object owner;

        public object Owner { get => owner; set => owner = value; }

        [SerializeField]
#if UNITY_EDITOR && ODIN_INSPECTOR
        [ShowIf(nameof(detailedInfo), Value = true)]
        [HideInEditorMode]
#endif
        private bool removeWhenDone;

        public bool RemoveWhenDone { get => removeWhenDone; set => removeWhenDone = value; }

        [SerializeField]
#if UNITY_EDITOR && ODIN_INSPECTOR
        [ShowIf(nameof(detailedInfo), Value = true)]
        [HideInEditorMode]
#endif

        private int updateOrder;
        public int UpdateOrder { get => updateOrder; set => updateOrder = value; }

        void ICommandInstant.Execute()
        {
            CommandState = COMMAND_STATE.EXECUTING;
            OnCommandStart?.Invoke(this);
            Execute();
            Success();
        }

        public abstract void Execute();

        /// <summary>
        /// Executed when the command ends with the status of success , defined in command and triggered automatically
        /// </summary>
        public virtual void OnSuccess() { }

        /// <summary>
        /// Executed when the command ends regardless of the status , defined in command and triggered automatically
        /// </summary>
        public virtual void OnEnd() { }

        public CommandInstant()
        {
            CommandState = COMMAND_STATE.WATING;
            FallbackCommand = null;
        }

        public ICommandBase SetFallbackCommand(ICommandBase fallback)
        {
            FallbackCommand = fallback;
            return this;
        }
        public ICommandBase GetExcutingCommand()
        {
            // waiting
            switch (CommandState)
            {
                case COMMAND_STATE.WATING:
                    return this;
                case COMMAND_STATE.EXECUTING:
                    return this;
                case COMMAND_STATE.SUCCESS:
                    return this;
                case COMMAND_STATE.INTERRUPTED:
                    return this;
                /// in this case default is COMMAND_STATE.FAILED
                default:
                    {
                        if (FallbackCommand == null)
                            return this;

                        // propagate to the fallback command
                        return FallbackCommand.GetExcutingCommand();
                    }
            }
        }

        /// <summary>
        /// Starts the command , called by ICommandBatch instances
        /// </summary>
        public void Start()
        {
            CommandState = COMMAND_STATE.EXECUTING;
            OnCommandStart?.Invoke(this);
            Execute();
            Success();
        }

        /// <summary>
        /// Ends the command , called by automatically (interal)
        /// </summary>
        private void End()
        {
            OnEnd();
            OnCommandEnd?.Invoke(this);
            OnCommandEndSpecific?.Invoke((T)this);
        }

        #region method to use from within the command to control its lifetime

        /// <summary>
        /// Ends the command with status of SUCCESS
        /// </summary>
        public void Success()
        {
            CommandState = COMMAND_STATE.SUCCESS;
            OnSuccess();
            End();
        }

        /// <summary>
        /// Ends the command with NO status and reset it to start over
        /// </summary>
        public void Reset()
        {
            // reset the parameters
            CommandState = COMMAND_STATE.EXECUTING;
        }

        public void OnStart()
        {
        }

        public void OnTick(float delta)
        {
        }

        public void Interrupt()
        {
        }
        #endregion

    }
    public abstract class CommandInstant<T, TResult> : CommandInstant<T>, IResult<TResult>, ICommandInstant<TResult> where T : CommandInstant<T, TResult>
    {
        public bool IsReady => CommandState == COMMAND_STATE.SUCCESS;
        public TResult Result { get; protected set; }
        protected abstract TResult GetResult();

        TResult ICommandInstant<TResult>.GetResult()
        {
            return GetResult();
        }

        public override void Execute()
        {
            Result = GetResult();
        }
    }
}
