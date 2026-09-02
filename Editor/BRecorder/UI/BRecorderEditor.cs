using Bloodthirst.Core.Utils;
using Bloodthirst.Runtime.BRecorder;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.EditorCoroutines.Editor;
using UnityEditor;
using UnityEditor.Callbacks;
using UnityEditor.UIElements;
using UnityEngine;
using UnityEngine.UIElements;

namespace Bloodthirst.Editor.BRecorder
{
    public class BRecorderEditor : EditorWindow
    {
        private const string PATH_UXML = "Packages/com.bloodthirst.bloodthirst-core/Editor/BRecorder/UI/BRecorderEditor.uxml";
        private const string PATH_USS = "Packages/com.bloodthirst.bloodthirst-core/Editor/BRecorder/UI/BRecorderEditor.uss";

        private const float PIXEL_PER_SECOND = 40f;

        private const float TIMELINE_HEIGHT = 200;

        private const string SETTINGS_KEY = "BRecorder_Settings";

        [MenuItem("Bloodthirst Tools/BRecorder")]
        public static void OpenEditor()
        {
            BRecorderEditor wnd = GetWindow<BRecorderEditor>();
            wnd.minSize = new Vector2(500, 500);
            wnd.titleContent = new GUIContent("BRecorder");
        }

        [OnOpenAsset(OnOpenAssetAttributeMode.Execute)]
        public static bool HandleOpenAsset(EntityId instanceID, int line)
        {
            UnityEngine.Object asset = EditorUtility.EntityIdToObject(instanceID);

            if (!(asset is BRecorderAsset rec))
                return false;

            BRecorderEditor wnd = GetWindow<BRecorderEditor>();
            wnd.minSize = new Vector2(500, 500);
            wnd.titleContent = new GUIContent("BRecorder");

            EditorCoroutineUtility.StartCoroutine(CrtOpenAssetAfterLayoutIsDone(wnd , rec) , wnd);

            return true;
        }


        private static IEnumerator WaitUntilLayoutDone(BRecorderEditor wnd)
        {
            while (!EditorUtils.IsLayoutBuilt(wnd.Root))
            {
                yield return null;
            }
        }

        private static IEnumerator CrtOpenAssetAfterLayoutIsDone( BRecorderEditor wnd, BRecorderAsset asset)
        {
            yield return WaitUntilLayoutDone(wnd);

            wnd.OpenSession.value = asset;
        }

        private VisualElement Root { get; set; }
        private Label GameTimer => Root.Q<Label>(nameof(GameTimer));
        private Toggle UpdateEveyframe => Root.Q<Toggle>(nameof(UpdateEveyframe));
        private Toggle ShowCurrentFrame => Root.Q<Toggle>(nameof(ShowCurrentFrame));
        private Toggle RecordOnGameStart => Root.Q<Toggle>(nameof(RecordOnGameStart));
        private VisualElement RecordBtn => Root.Q<VisualElement>(nameof(RecordBtn));
        private VisualElement PlayBtn => Root.Q<VisualElement>(nameof(PlayBtn));
        private VisualElement StopBtn => Root.Q<VisualElement>(nameof(StopBtn));
        private VisualElement PauseBtn => Root.Q<VisualElement>(nameof(PauseBtn));
        private VisualElement ClearBtn => Root.Q<VisualElement>(nameof(ClearBtn));
        internal ObjectField OpenSession => Root.Q<ObjectField>(nameof(OpenSession));
        private VisualElement ClearAssetBtn => Root.Q<VisualElement>(nameof(ClearAssetBtn));
        private VisualElement SaveBtn => Root.Q<VisualElement>(nameof(SaveBtn));
        private VisualElement OpenBtn => Root.Q<VisualElement>(nameof(OpenBtn));
        private VisualElement Timeline => Root.Q<VisualElement>(nameof(Timeline));
        private VisualElement TimeAxis => Root.Q<VisualElement>(nameof(TimeAxis));
        private VisualElement CursorsContainer => Root.Q<VisualElement>(nameof(CursorsContainer));
        private VisualElement TimelineContainer => Root.Q<VisualElement>(nameof(TimelineContainer));
        private Scroller HorizontalScroller => Root.Q<Scroller>(nameof(HorizontalScroller));
        internal bool StartOnGameStart => RecordOnGameStart.value;

        private List<BRecorderCursorUICommand> Cursors { get; set; }
        private BRecorderCursorUIFrame CurrentFrameCursor { get; set; }
        private List<BRecorderInfoUI> Infos { get; set; }

        #region scroll

        private Vector2 horizontalScrollMinMax;

        private float horizontalScrollValue;
        internal float HorizontalScrollValue
        {
            get => horizontalScrollValue;
            set
            {
                horizontalScrollValue = Mathf.Clamp(value, horizontalScrollMinMax.x, horizontalScrollMinMax.y);
                RepaintViewport();
                RepaintScroller();
                RepaintTimeAxis();
                RefreshTimeline();
            }
        }
        #endregion

        #region zoom

        private Vector2 horizontalZoomMinMax;
        private float currentHorizontalZoom;

        /// <summary>
        /// <para> if equal to 1 means the zoom is neutral </para>
        /// <para> if more than 1 means it's zoomed in </para>
        /// <para> if less than 1 means it's zoomed out</para>
        /// </summary>
        internal float CurrentHorizontalZoom
        {
            get => currentHorizontalZoom;
            set
            {
                currentHorizontalZoom = Mathf.Clamp(value, horizontalZoomMinMax.x, horizontalZoomMinMax.y);
                RepaintViewport();
                RepaintScroller();
                RepaintTimeAxis();
                RefreshTimeline();
            }
        }
        #endregion

        internal Vector2 WorldMousePostion { get; set; }

        internal bool UpdatePerFrame { get; set; }

        private List<BRecorderActionBase> Actions { get; set; }
        public BEventSystem.BEventSystem<BRecorderEventBase> EventSystem { get; private set; }

        private void OnEnable()
        {
            EditorApplication.update -= HandleOnUpdate;
            EditorApplication.update += HandleOnUpdate;
        }

        private void OnDisable()
        {
            EditorApplication.update -= HandleOnUpdate;
        }

        private void HandleOnUpdate()
        {
            if (!isLoaded)
                return;

            UpdateTimer();

            UpdateTimelinePerFrame();

            ShowCurrentFrameCursor();
        }

        private void UpdateTimer()
        {
            if (EditorApplication.isCompiling)
                return;

            if (!Application.isPlaying)
            {
                GameTimer.text = "In Editor";
                return;
            }

            TimeSpan t = new TimeSpan(0, 0, (int)Time.time);

            DateTime asDate = new DateTime(t.Ticks);

            GameTimer.text = asDate.ToString("mm:ss");
        }

        private void UpdateTimelinePerFrame()
        {
            if (!Application.isPlaying)
                return;

            if (!UpdateEveyframe.value)
                return;

            CurrentHorizontalZoom = ZoomNeededToShowUntil(Time.time);
        }

        private void ShowCurrentFrameCursor()
        {
            if (!ShowCurrentFrame.value || !Application.isPlaying)
            {
                CurrentFrameCursor.Display(false);
                return;
            }

            CurrentFrameCursor.Display(true);

            CurrentFrameCursor.style.left = OffsetPositionFromGameTime(Time.time);
        }

        internal float ZoomNeededToShowUntil(float gameTime)
        {
            float extraPaddingToTheRight = 50f;
            float containerWidth = TimelineContainer.resolvedStyle.width;
            float contentWidth = (gameTime * PIXEL_PER_SECOND) + extraPaddingToTheRight;

            float zoomNeeded = containerWidth / contentWidth;

            return zoomNeeded;
        }

        private bool isLoaded;

        public void CreateGUI()
        {
            isLoaded = false;

            // Each editor window contains a root VisualElement object
            Root = rootVisualElement;

            // Import
            StyleSheet styleSheet = AssetDatabase.LoadAssetAtPath<StyleSheet>(PATH_USS);
            VisualTreeAsset visualTree = AssetDatabase.LoadAssetAtPath<VisualTreeAsset>(PATH_UXML);
            visualTree.CloneTree(Root);
            if (!Root.styleSheets.Contains(EditorConsts.GlobalStyleSheet))
            {
                Root.styleSheets.Add(EditorConsts.GlobalStyleSheet);
            }
            Root.styleSheets.Add(styleSheet);

            EditorCoroutineUtility.StartCoroutine(CrtDelayedInit(), this);
        }

        private IEnumerator CrtDelayedInit()
        {
            isLoaded = false;

            Initialize();
            InitUI();
            ListenUI();

            yield return WaitUntilLayoutDone(this);

            // refresh
            RepaintViewport();
            RepaintScroller();
            RepaintTimline();

            isLoaded = true;

            // settings
            LoadSettings();
        }
        private void Initialize()
        {
            EventSystem = new BEventSystem.BEventSystem<BRecorderEventBase>();
            Actions = new List<BRecorderActionBase>();

            List<Type> actionTypes = TypeUtils.AllTypes
                .Where(t => t.IsClass)
                .Where(t => !t.IsAbstract)
                .Where(t => TypeUtils.IsSubTypeOf(t, typeof(BRecorderActionBase)))
                .ToList();

            foreach (Type t in actionTypes)
            {
                BRecorderActionBase action = (BRecorderActionBase)Activator.CreateInstance(t, args: this);
                action.Initialize();
                Actions.Add(action);
            }
        }

        private void InitUI()
        {
            // cursors
            Cursors = new List<BRecorderCursorUICommand>();

            // current frame cursor
            CurrentFrameCursor = new BRecorderCursorUIFrame();
            CurrentFrameCursor.Display(false);

            // infos
            Infos = new List<BRecorderInfoUI>();

            // zoom
            horizontalZoomMinMax = new Vector2(0.1f, 10f);
            currentHorizontalZoom = 1;

            // open asset
            OpenSession.objectType = typeof(BRecorderAsset);
            OpenSession.SetEnabled(false);
            //BRecorderRuntime.CurrentSession = OpenAsset?.Session;



            // playing/recording state
            RefreshState();

            // refresh
            RepaintViewport();
            RepaintScroller();
            RepaintTimline();
            
        }

        private void ListenUI()
        {
            BRecorderRuntime.OnStateChanged += HandleStateChanged;
            BRecorderRuntime.OnSessionChanged += HandleSessionChanged;

            TimelineContainer.RegisterCallback<GeometryChangedEvent>(HandleWindowSizeChanged);

            PlayBtn.RegisterCallback<ClickEvent>(HandlePlayClicked);
            RecordBtn.RegisterCallback<ClickEvent>(HandleRecordClicked);
            StopBtn.RegisterCallback<ClickEvent>(HandleStopClicked);
            PauseBtn.RegisterCallback<ClickEvent>(HandlePauseClicked);
            ClearBtn.RegisterCallback<ClickEvent>(HandleClearClicked);

            OpenSession.RegisterValueChangedCallback(HandleOpenAssetChanged);
            ClearAssetBtn.RegisterCallback<ClickEvent>(HandleClearAssetClicked);


            OpenBtn.RegisterCallback<ClickEvent>(HandleOpenClicked);
            SaveBtn.RegisterCallback<ClickEvent>(HandleSaveClicked);

            Root.RegisterCallback<MouseMoveEvent>(HandleEditorMouseMove);
            Root.RegisterCallback<KeyDownEvent>(HandleEditorKeydown);
            Timeline.RegisterCallback<WheelEvent>(HandleScrollWheel);
            Timeline.RegisterCallback<MouseDownEvent>(HandleMouseDown);
            Timeline.RegisterCallback<MouseMoveEvent>(HandleMouseMove);
            Timeline.RegisterCallback<MouseUpEvent>(HandleMouseUp);
            Timeline.RegisterCallback<MouseLeaveEvent>(HandleMouseLeave);
            HorizontalScroller.RegisterCallback<ChangeEvent<float>>(HandleScrollChanged);
        }

        private float InvertScrollValue(float val)
        {
            float reversed_t = Mathf.InverseLerp(HorizontalScroller.lowValue, HorizontalScroller.highValue, val);

            float correct_val = Mathf.Lerp(HorizontalScroller.lowValue, HorizontalScroller.highValue, 1 - reversed_t);

            return correct_val;
        }

        private void HandleScrollChanged(ChangeEvent<float> evt)
        {
            float correct_val = InvertScrollValue(evt.newValue);

            EventSystem.Trigger(new OnTimelineHorizontalScrollChanged(this, correct_val));
        }

        private void HandleWindowSizeChanged(GeometryChangedEvent evt)
        {
            RepaintViewport();
            RepaintScroller();
            RepaintTimeAxis();
            RefreshTimeline();
        }

        private void HandleSessionChanged(BRecorderSession oldSession, BRecorderSession newSession)
        {
            EventSystem.Trigger(new OnTimelineSessionChanged(this, oldSession, newSession));
        }

        private void HandleStateChanged(BRecorderRuntime.RECORDER_STATE oldState, BRecorderRuntime.RECORDER_STATE newState)
        {
            EventSystem.Trigger(new OnTimelineStateChanged(this, oldState, newState));
        }

        internal void RepaintViewport()
        {
            // height
            Timeline.style.height = TIMELINE_HEIGHT;

            // width
            float lastCommandTime = 0;

            if (BRecorderRuntime.CurrentSession != null && BRecorderRuntime.CurrentSession.Commands.Count > 0)
            {
                lastCommandTime = BRecorderRuntime.CurrentSession.Commands.Last().GameTime;
            }

            float extraPaddingToTheRight = 50f;
            float defaultWidth = TimelineContainer.resolvedStyle.width;
            float contentWidth = (lastCommandTime * PIXEL_PER_SECOND * CurrentHorizontalZoom) + extraPaddingToTheRight;
            float neededWidth = Mathf.Max(defaultWidth, contentWidth);

            float widthInStyle = neededWidth;

            Timeline.style.width = widthInStyle;
        }

        internal void RepaintScrollerAndKeepSamePlace()
        {
            float lastCommandTime = 0;

            if (BRecorderRuntime.CurrentSession != null && BRecorderRuntime.CurrentSession.Commands.Count > 0)
            {
                lastCommandTime = BRecorderRuntime.CurrentSession.Commands.Last().GameTime;
            }

            float extraPaddingToTheRight = 50f;
            float defaultWidth = TimelineContainer.resolvedStyle.width;
            float contentWidth = (lastCommandTime * PIXEL_PER_SECOND * CurrentHorizontalZoom) + extraPaddingToTheRight;
            float neededWidth = Mathf.Max(defaultWidth, contentWidth);

            float difference = neededWidth - defaultWidth;

            float oldValue = HorizontalScrollValue;

            if (difference > 0)
            {
                horizontalScrollMinMax.x = 0;
                horizontalScrollMinMax.y = difference;

                HorizontalScroller.SetEnabled(true);
            }
            else
            {
                horizontalScrollMinMax.x = 0;
                horizontalScrollMinMax.y = 0;

                HorizontalScroller.SetEnabled(false);
            }

            // keep the same scroller ratio
            HorizontalScroller.lowValue = horizontalScrollMinMax.x;
            HorizontalScroller.highValue = horizontalScrollMinMax.y;

            horizontalScrollValue = oldValue;
            HorizontalScroller.slider.SetValueWithoutNotify(InvertScrollValue(oldValue));
        }

        internal void RepaintScroller()
        {
            float lastCommandTime = 0;

            if (BRecorderRuntime.CurrentSession != null && BRecorderRuntime.CurrentSession.Commands.Count > 0)
            {
                lastCommandTime = BRecorderRuntime.CurrentSession.Commands.Last().GameTime;
            }

            float extraPaddingToTheRight = 50f;
            float defaultWidth = TimelineContainer.resolvedStyle.width;
            float contentWidth = (lastCommandTime * PIXEL_PER_SECOND * CurrentHorizontalZoom) + extraPaddingToTheRight;
            float neededWidth = Mathf.Max(defaultWidth, contentWidth);

            float difference = neededWidth - defaultWidth;

            Vector2 oldMinMax = horizontalScrollMinMax;
            float oldValue = HorizontalScrollValue;

            float old_t = Mathf.InverseLerp(oldMinMax.x, oldMinMax.y, oldValue);

            if (difference > 0)
            {
                horizontalScrollMinMax.x = 0;
                horizontalScrollMinMax.y = difference;

                HorizontalScroller.SetEnabled(true);
            }
            else
            {
                horizontalScrollMinMax.x = 0;
                horizontalScrollMinMax.y = 0;

                HorizontalScroller.SetEnabled(false);
            }

            // keep the same scroller ratio
            HorizontalScroller.lowValue = horizontalScrollMinMax.x;
            HorizontalScroller.highValue = horizontalScrollMinMax.y;

            horizontalScrollValue = Mathf.Lerp(horizontalScrollMinMax.x, horizontalScrollMinMax.y, old_t);
            HorizontalScroller.slider.SetValueWithoutNotify(InvertScrollValue(horizontalScrollValue));
        }

        internal void RepaintTimeAxis()
        {
            TimeAxis.Clear();

            // pixel per one second
            float PixelsPerStep = PIXEL_PER_SECOND * CurrentHorizontalZoom;

            // maximum seconds we can show on the axis
            float totalStepsOnAxis = TimeAxis.resolvedStyle.width / PixelsPerStep;

            float labelSize = 40f;

            // maximum amount of labels to show in the axis
            float totalLabelsToShow = TimeAxis.resolvedStyle.width / labelSize;

            int secondPerStep = 1;

            float stepCount = totalStepsOnAxis;

            if (totalStepsOnAxis > totalLabelsToShow)
            {
                while (stepCount > totalLabelsToShow)
                {
                    secondPerStep++;
                    stepCount = totalStepsOnAxis / secondPerStep;
                }
            }

            for (int i = 0; i < stepCount; i++)
            {
                int seconds = i * secondPerStep;

                Label timeLabel = new Label();
                timeLabel.name = $"sec-{seconds}";
                timeLabel.text = $"{seconds}";
                timeLabel.style.width = labelSize;

                TimeAxis.Add(timeLabel);

                timeLabel.style.position = new StyleEnum<Position>(Position.Absolute);
                timeLabel.style.left = (i * secondPerStep * PixelsPerStep);
            }
        }

        internal void RepaintTimline()
        {
            Timeline.style.left = -HorizontalScrollValue;

            CursorsContainer.Clear();
            Cursors.Clear();
            Infos.Clear();

            CursorsContainer.Add(CurrentFrameCursor);

            if (BRecorderRuntime.CurrentSession == null)
                return;

            for (int i = 0; i < BRecorderRuntime.CurrentSession.Commands.Count; i++)
            {
                IBRecorderCommand cmd = BRecorderRuntime.CurrentSession.Commands[i];

                BRecorderCursorUICommand cmdCursor = new BRecorderCursorUICommand(cmd);

                CursorsContainer.Add(cmdCursor);
                Cursors.Add(cmdCursor);


                float offset = OffsetPositionFromGameTime(cmd.GameTime);
                cmdCursor.style.left = offset;
            }
        }

        internal void RefreshTimeline()
        {
            Timeline.style.left = -HorizontalScrollValue;

            if (BRecorderRuntime.CurrentSession == null)
                return;

            for (int i = 0; i < Cursors.Count; i++)
            {
                BRecorderCursorUICommand ui = Cursors[i];
                ui.style.left = OffsetPositionFromGameTime(ui.Command.GameTime);
            }
        }

        internal void AddCommand(IBRecorderCommand cmd)
        {
            BRecorderCursorUICommand cmdCursor = new BRecorderCursorUICommand(cmd);

            CursorsContainer.Add(cmdCursor);
            Cursors.Add(cmdCursor);

            float offset = OffsetPositionFromGameTime(cmd.GameTime);
            cmdCursor.style.left = offset;
        }

        internal float OffsetPositionFromGameTime(float gameTime)
        {
            return gameTime * PIXEL_PER_SECOND * CurrentHorizontalZoom;
        }

        internal void RemoveCommand(IBRecorderCommand cmd)
        {
            for (int i = 0; i < Cursors.Count; i++)
            {
                if (Cursors[i].Command != cmd)
                    continue;

                BRecorderCursorUIBase ui = Cursors[i];
                CursorsContainer.RemoveAt(i);
                Cursors.RemoveAt(i);
                return;
            }
        }

        internal void RefreshState()
        {
            BRecorderRuntime.RECORDER_STATE recorderState = BRecorderRuntime.RecordingState;

            switch (recorderState)
            {
                case BRecorderRuntime.RECORDER_STATE.PLAYING:
                    {
                        PauseBtn.SetEnabled(true);
                        StopBtn.SetEnabled(true);

                        PlayBtn.SetEnabled(false);
                        RecordBtn.SetEnabled(false);
                        break;
                    }
                case BRecorderRuntime.RECORDER_STATE.PAUSED:
                    {
                        PlayBtn.SetEnabled(true);
                        StopBtn.SetEnabled(true);

                        PauseBtn.SetEnabled(false);
                        RecordBtn.SetEnabled(false);

                        break;
                    }
                case BRecorderRuntime.RECORDER_STATE.RECORDING:
                    {
                        StopBtn.SetEnabled(true);
                        PauseBtn.SetEnabled(false);

                        PlayBtn.SetEnabled(false);
                        RecordBtn.SetEnabled(false);
                        break;
                    }
                case BRecorderRuntime.RECORDER_STATE.IDLE:
                    {
                        if (OpenSession.value != null)
                        {
                            PlayBtn.SetEnabled(true);
                        }
                        else
                        {
                            PlayBtn.SetEnabled(false);
                        }

                        RecordBtn.SetEnabled(true);

                        PauseBtn.SetEnabled(false);
                        StopBtn.SetEnabled(false);
                        break;
                    }
            }
        }

        private void HandleOpenClicked(ClickEvent evt)
        {
            EventSystem.Trigger(new OnTimelineOpen(this));
        }

        private void HandleSaveClicked(ClickEvent evt)
        {
            EventSystem.Trigger(new OnTimelineSave(this));
        }

        private void HandlePauseClicked(ClickEvent evt)
        {
            BRecorderRuntime.PauseSession();
        }

        private void HandleClearClicked(ClickEvent evt)
        {
            BRecorderRuntime.CurrentSession = new BRecorderSession();
        }

        private void HandleStopClicked(ClickEvent evt)
        {
            BRecorderRuntime.StopRecording();
        }

        private void HandleRecordClicked(ClickEvent evt)
        {
            BRecorderRuntime.StartRecording();
        }

        private void HandlePlayClicked(ClickEvent evt)
        {
            BRecorderRuntime.PlaySession();
        }

        private void HandleOpenAssetChanged(ChangeEvent<UnityEngine.Object> evt)
        {
            BRecorderAsset asset = (BRecorderAsset)evt.newValue;
            BRecorderRuntime.CurrentSession = asset?.Session;

            RepaintViewport();
            RepaintScroller();
            RepaintTimeAxis();
            RepaintTimline();
            RefreshState();
        }

        private void HandleClearAssetClicked(ClickEvent evt)
        {
            OpenSession.value = null;
        }

        private void HandleMouseLeave(MouseLeaveEvent evt)
        {
            EventSystem.Trigger(new OnTimelineMouseLeave(this, evt));
        }

        private void HandleEditorMouseMove(MouseMoveEvent evt)
        {
            Vector2 localPos = evt.localMousePosition;

            WorldMousePostion = Root.LocalToWorld(localPos);
        }

        private void HandleScrollWheel(WheelEvent evt)
        {
            EventSystem.Trigger(new OnTimelineScrollWheel(this, evt));
        }

        private void HandleEditorKeydown(KeyDownEvent evt)
        {
            EventSystem.Trigger(new OnTimelineKeydown(this, evt));
        }

        private void HandleMouseUp(MouseUpEvent evt)
        {
            EventSystem.Trigger(new OnTimelineMouseUp(this, evt));
        }

        private void HandleMouseMove(MouseMoveEvent evt)
        {
            EventSystem.Trigger(new OnTimelineMouseMove(this, evt));
        }

        private void HandleMouseDown(MouseDownEvent evt)
        {
            EventSystem.Trigger(new OnTimelineMouseDown(this, evt));
        }

        internal BRecorderState GetState()
        {
            BRecorderState defaultState = new BRecorderState();
            defaultState.openAsset = GlobalObjectId.GetGlobalObjectIdSlow(null).ToString();

            string defaultVal = JsonUtility.ToJson(defaultState);
            string json = SessionState.GetString(SETTINGS_KEY , defaultVal);

            Debug.Log($"Get State : {json}");

            BRecorderState settings = JsonUtility.FromJson<BRecorderState>(json);

            return settings;
        }

        internal void SetState(BRecorderState recorderState)
        {
            string json = JsonUtility.ToJson(recorderState);

            Debug.Log($"Set State : {json}");

            SessionState.SetString(SETTINGS_KEY, json);
        }

        internal void SaveSettings()
        {
            BRecorderState curr = new BRecorderState()
            {
                openAsset = GlobalObjectId.GetGlobalObjectIdSlow(OpenSession.value).ToString(),
                recorderState = BRecorderRuntime.RecordingState,
                updateEveyframe_Value = UpdateEveyframe.value,
                recordOnGameStart_Value = RecordOnGameStart.value,
                showCurrentFrame_Value = ShowCurrentFrame.value,
            };

            SetState(curr);
        }

        internal void LoadSettings()
        {
            BRecorderState settings = GetState();

            switch (settings.recorderState)
            {
                case BRecorderRuntime.RECORDER_STATE.IDLE:
                    {
                        BRecorderRuntime.StopRecording();
                        break;
                    }
                case BRecorderRuntime.RECORDER_STATE.PAUSED:
                    {
                        BRecorderRuntime.PauseSession();
                        break;
                    }
                case BRecorderRuntime.RECORDER_STATE.PLAYING:
                    {
                        BRecorderRuntime.PlaySession();
                        settings.updateEveyframe_Value = true;
                        settings.showCurrentFrame_Value = true;
                        break;
                    }
                case BRecorderRuntime.RECORDER_STATE.RECORDING:
                    {
                        BRecorderRuntime.SetRecording();

                        if(BRecorderRuntime.CurrentSession == null)
                        {
                            BRecorderRuntime.CurrentSession = new BRecorderSession();
                        }

                        settings.updateEveyframe_Value = true;
                        settings.showCurrentFrame_Value = true;
                        break;
                    }
            }

            GlobalObjectId.TryParse(settings.openAsset, out GlobalObjectId id);
            BRecorderAsset asset = (BRecorderAsset)GlobalObjectId.GlobalObjectIdentifierToObjectSlow(id);

            OpenSession.value = asset;
            UpdateEveyframe.value = settings.updateEveyframe_Value;
            RecordOnGameStart.value = settings.recordOnGameStart_Value;
            ShowCurrentFrame.value = settings.showCurrentFrame_Value;
        }

        internal void ResetSettings()
        {
            BRecorderState curr = new BRecorderState()
            {
                openAsset = GlobalObjectId.GetGlobalObjectIdSlow(null).ToString(),
                recorderState = BRecorderRuntime.RECORDER_STATE.IDLE,
                updateEveyframe_Value = false,
                recordOnGameStart_Value = false,
                showCurrentFrame_Value = false,
            };

            SetState(curr);

            BRecorderRuntime.StopRecording();
            UpdateEveyframe.value = false;
            RecordOnGameStart.value = false;
            ShowCurrentFrame.value = false;
        }

        private void Destroy()
        {
            // ui
            BRecorderRuntime.OnStateChanged -= HandleStateChanged;
            BRecorderRuntime.OnSessionChanged -= HandleSessionChanged;

            Timeline.UnregisterCallback<GeometryChangedEvent>(HandleWindowSizeChanged);
            Root.UnregisterCallback<KeyDownEvent>(HandleEditorKeydown);

            PlayBtn.UnregisterCallback<ClickEvent>(HandlePlayClicked);
            RecordBtn.UnregisterCallback<ClickEvent>(HandleRecordClicked);
            StopBtn.UnregisterCallback<ClickEvent>(HandleStopClicked);
            PauseBtn.UnregisterCallback<ClickEvent>(HandlePauseClicked);
            ClearBtn.UnregisterCallback<ClickEvent>(HandleClearClicked);

            OpenSession.UnregisterValueChangedCallback(HandleOpenAssetChanged);
            ClearAssetBtn.UnregisterCallback<ClickEvent>(HandleClearAssetClicked);

            OpenBtn.UnregisterCallback<ClickEvent>(HandleOpenClicked);
            SaveBtn.UnregisterCallback<ClickEvent>(HandleSaveClicked);

            Root.UnregisterCallback<MouseMoveEvent>(HandleEditorMouseMove);
            Timeline.UnregisterCallback<WheelEvent>(HandleScrollWheel);
            Timeline.UnregisterCallback<MouseDownEvent>(HandleMouseDown);
            Timeline.UnregisterCallback<MouseMoveEvent>(HandleMouseMove);
            Timeline.UnregisterCallback<MouseUpEvent>(HandleMouseUp);
            Timeline.UnregisterCallback<MouseLeaveEvent>(HandleMouseLeave);
            HorizontalScroller.RegisterCallback<ChangeEvent<float>>(HandleScrollChanged);

            // events
            EventSystem.UnlistenAll();

            // actions
            foreach (BRecorderActionBase action in Actions)
            {
                action.Destroy();
            }

            Actions.Clear();
        }

        private void OnDestroy()
        {
            SaveSettings();
            Destroy();
        }
    }
}