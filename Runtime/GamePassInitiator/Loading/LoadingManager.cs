using Bloodthirst.Core.Setup;
using Bloodthirst.System.CommandSystem;
#if ODIN_INSPECTOR
using Sirenix.OdinInspector;
#endif
using System;
using System.Collections.Generic;
#if UNITY_EDITOR
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
#endif
using UnityEngine;

namespace Bloodthirst.Core.SceneManager
{
    public enum LOADDING_STATE
    {
        FREE,
        LOADING
    }

    public class LoadingManager : MonoBehaviour
#if UNITY_EDITOR
        , IPreprocessBuildWithReport
#endif
    {
#if UNITY_EDITOR
        public int callbackOrder => 100;

        public void OnPreprocessBuild(BuildReport report)
        {
            ScenesListData.Instance.InitializeScenes();
        }
#endif
        public event Action<LoadingManager> OnLoadingProgressChanged;

        public event Action<LoadingManager> OnLoadingStatusChanged;

        [SerializeField]
        private float progress;

        [SerializeField]
        private LOADDING_STATE state;

        [SerializeField]
        private bool showLoading;

        public bool ShowLoading => showLoading;
        /// <summary>
        /// Progress value between 0 -> 1
        /// </summary>
        public float Progress
        {
            get => progress;
            private set
            {
                progress = value;
            }
        }

        public LOADDING_STATE State
        {
            get => state;
            private set
            {
                state = value;
            }
        }


#if ODIN_INSPECTOR
        [ShowInInspector]
#endif

        private List<AsynOperationInternalHandle> pendingCommands;

#if ODIN_INSPECTOR
        [ShowInInspector]
#endif
        private List<AsynOperationInternalHandle> doneCommands;

#if ODIN_INSPECTOR
        [ShowInInspector]
#endif
        private AsynOperationInternalHandle currentOperation;

        private CommandManager commandManager;

        public IProgressCommand CurrentTask()
        {
            return currentOperation?.command;
        }

        public void Initialize(ICommandManagerProvider commandManagerProvider)
        {
            State = LOADDING_STATE.FREE;
            Progress = 0;

            pendingCommands = new List<AsynOperationInternalHandle>();
            doneCommands = new List<AsynOperationInternalHandle>();
            currentOperation = null;

            commandManager = commandManagerProvider.Get();
        }

        public void Interrupt()
        {
            if (currentOperation == null)
                return;

            currentOperation.command.Interrupt();
            currentOperation.handle.IsDone = true;
            currentOperation = null;
        }

        private void Update()
        {
            if(pendingCommands == null) { return; }

            // if nothing to run
            if (pendingCommands.Count == 0 && currentOperation == null)
            {
                doneCommands.Clear();
                RefreshState();
                return;
            }

            // check if current operation is done
            if (currentOperation != null)
            {
                if (!currentOperation.command.IsDone())
                {
                    RefreshState();
                    return;
                }


                currentOperation.handle.IsDone = true;
                doneCommands.Add(currentOperation);
                currentOperation = null;
            }

            // try to run the next command if needed
            while (pendingCommands.Count != 0 && currentOperation == null)
            {
                AsynOperationInternalHandle curr = pendingCommands[0];
                pendingCommands.RemoveAt(0);

                if (!curr.handle.Operation.ShouldExecute())
                {
                    curr.handle.IsDone = true;
                }
                else
                {
                    currentOperation = curr;
                    showLoading = curr.ShowLoadingScreen;

                    IProgressCommand cmd = curr.handle.Operation.CreateOperation();
                    curr.command = cmd;
                    commandManager.AppendCommand(this, cmd, true);
                }
            }

            RefreshState();
        }

        private void RefreshState()
        {
            if (pendingCommands.Count == 0 && currentOperation == null)
            {
                SetStateAndProgress(LOADDING_STATE.FREE, 0);
                return;
            }

            float progressSum = doneCommands.Count;
            float totalOpsSum = pendingCommands.Count + doneCommands.Count;

            if (currentOperation != null)
            {
                progressSum += currentOperation.command.CurrentProgress;
                totalOpsSum++;
            }

            float progressPerecentage = progressSum / totalOpsSum;

            SetStateAndProgress(LOADDING_STATE.LOADING, progressPerecentage);
        }

        private void SetStateAndProgress(LOADDING_STATE state, float progress)
        {
            bool stateChanged = false;
            bool progressChanged = false;
            if (state != State)
            {
                stateChanged = true;
                State = state;
            }

            if (progress != Progress)
            {
                progressChanged = true;
                Progress = progress;
            }

            if (stateChanged)
            {
                OnLoadingStatusChanged?.Invoke(this);
            }

            if (progressChanged)
            {
                OnLoadingProgressChanged?.Invoke(this);
            }
        }

        public AsyncOperationHandle RunAsyncTask(IAsynOperationWrapper asyncOperations)
        {
            AsyncOperationHandle handle = new AsyncOperationHandle()
            {
                Operation = asyncOperations,
                IsDone = false
            };

            AsynOperationInternalHandle internalHandle = new AsynOperationInternalHandle()
            {
                command = null,
                handle = handle,
                ShowLoadingScreen = asyncOperations.ShowLoadingScreen
            };

            pendingCommands.Add(internalHandle);

            return handle;
        }


        private class AsynOperationInternalHandle
        {
            public AsyncOperationHandle handle;
            public IProgressCommand command;
            public bool ShowLoadingScreen;
        }
    }

    public class AsyncOperationHandle
    {
        public IAsynOperationWrapper Operation;
        public bool IsDone;
    }



}