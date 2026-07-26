using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;
using Bloodthirst.Editor;
using System;
using Bloodthirst.Core.BISDSystem;
using System.Collections.Generic;

namespace Bloodthirst.Core.BISD.Editor
{
    [InitializeOnLoad]
    public class BISDSaveLoadManager : EditorWindow
    {
        private const string PATH_USS = EditorConsts.GLOBAL_EDITOR_FOLRDER_PATH + "BISD/BISD SaveLoad/BISDSaveLoadManager.uss";
        private const string PATH_UXML = EditorConsts.GLOBAL_EDITOR_FOLRDER_PATH + "BISD/BISD SaveLoad/BISDSaveLoadManager.uxml";

        [MenuItem("Bloodthirst Tools/BISD Pattern/Save & Load Manager")]
        public static void ShowExample()
        {
            BISDSaveLoadManager wnd = GetWindow<BISDSaveLoadManager>();
            wnd.titleContent = new GUIContent("BISDSaveLoadManager");
        }

        private event Action OnSettingsChanged;

        private VisualElement root;
        private SaveLoadManager saveLoadManager;

        private VisualElement Root => root;

        private Button selectPathBtn => Root.Q<Button>(nameof(selectPathBtn));

        private TextField selectPathTxt => Root.Q<TextField>(nameof(selectPathTxt));

        private TextField titleTxt => Root.Q<TextField>(nameof(titleTxt));

        private Button createAssetBtn => Root.Q<Button>(nameof(createAssetBtn));

        static BISDSaveLoadManager()
        {
#if UNITY_EDITOR
            if (!EditorConsts.ON_ASSEMBLY_RELOAD_BISD_SAVE_MANAGER)
                return;
#endif
        }


        public void CreateGUI()
        {
            Init();

            selectPathTxt.SetEnabled(false);

            selectPathBtn.clickable.clicked -= OnSelectPathClicked;
            selectPathBtn.clickable.clicked += OnSelectPathClicked;

            createAssetBtn.clickable.clicked -= OnCreateAssetClicked;
            createAssetBtn.clickable.clicked += OnCreateAssetClicked;

            OnSettingsChanged -= OnSettingsChangedTriggered;
            OnSettingsChanged += OnSettingsChangedTriggered;

            EditorApplication.playModeStateChanged -= EditorApplication_playModeStateChanged;
            EditorApplication.playModeStateChanged += EditorApplication_playModeStateChanged;

            if (!EditorConsts.ON_ASSEMBLY_RELOAD_BISD_SAVE_MANAGER)
            {
                EditorUtility.DisplayDialog("Error", "Enable Assembly reload for BISD save manager", "Ok");
            }
        }

        private void OnAssetNameChanged(ChangeEvent<string> evt)
        {
            Refresh();
        }

        private void EditorApplication_playModeStateChanged(PlayModeStateChange obj)
        {
            Refresh();
        }

        private void Refresh()
        {
            createAssetBtn.SetEnabled(CanSave());
        }

        private bool CanSave()
        {
            if (selectPathTxt.value == string.Empty)
                return false;

            if (EditorApplication.isPaused || EditorApplication.isPlaying)
                return true;

            return false;
        }

        private void OnSettingsChangedTriggered()
        {
            Refresh();
        }

        private void OnCreateAssetClicked()
        {
            GameStateSaveInstance data = CreateInstance<GameStateSaveInstance>();

            data.Title = titleTxt.value;
            data.GameDatas = new List<SavedEntityEntry>();

            saveLoadManager.SaveRuntimeState(data.GameDatas);

            AssetDatabase.CreateAsset(data, selectPathTxt.value);

            GameStateSaveInstance created = AssetDatabase.LoadAssetAtPath<GameStateSaveInstance>(selectPathTxt.value);

            ProjectWindowUtil.ShowCreatedAsset(created);


        }

        private void OnSelectPathClicked()
        {
            string path = EditorUtility.SaveFilePanel("Select path to save the asset", "Assets", "GameData Save", "asset");

            if (string.IsNullOrEmpty(path))
                return;

            selectPathTxt.value = FileUtil.GetProjectRelativePath(path);

            OnSettingsChanged?.Invoke();
        }

        private void Init()
        {
            // Each editor window contains a root VisualElement object
            root = rootVisualElement;

            // Import UXML
            VisualTreeAsset visualTree = AssetDatabase.LoadAssetAtPath<VisualTreeAsset>(PATH_UXML);
            VisualElement uxml = visualTree.Instantiate();


            // A stylesheet can be added to a VisualElement.
            // The style will be applied to the VisualElement and all of its children.
            StyleSheet styleSheet = AssetDatabase.LoadAssetAtPath<StyleSheet>(PATH_USS);
            uxml.styleSheets.Add(EditorConsts.GlobalStyleSheet);
            uxml.styleSheets.Add(styleSheet);
            root.Add(uxml);

            saveLoadManager = new SaveLoadManager();
            saveLoadManager.Initialize();
        }
    }
}