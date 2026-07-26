using Bloodthirst.Editor.AssetProcessing;
using Bloodthirst.Core.Utils;
using Bloodthirst.Editor;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.EditorCoroutines.Editor;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEditor.UIElements;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UIElements;
using static UnityEngine.EventSystems.EventTrigger;

public class SceneEntry : VisualElement
{
    private const string UXML_PATH = EditorConsts.GLOBAL_EDITOR_FOLRDER_PATH + "Scene Load Helper/SceneEntry.uxml";
    private SceneAsset sceneAsset;
    private string scenePath;
    public string ScenePath => scenePath;

    private ObjectField SceneAsset => this.Q<ObjectField>(nameof(SceneAsset));
    private Toggle IsOpen => this.Q<Toggle>(nameof(IsOpen));
    private Button LoadSingleBtn => this.Q<Button>(nameof(LoadSingleBtn));
    private Button LoadAdditivelyBtn => this.Q<Button>(nameof(LoadAdditivelyBtn));
    private Button UnloadBtn => this.Q<Button>(nameof(UnloadBtn));

    public event Action<SceneEntry> OnLoadAdditive;
    public event Action<SceneEntry> OnLoadSingle;
    public event Action<SceneEntry> OnUnload;

    public SceneEntry()
    {
        VisualTreeAsset visualTree = AssetDatabase.LoadAssetAtPath<VisualTreeAsset>(UXML_PATH);

        visualTree.CloneTree(this);

        if (!styleSheets.Contains(EditorConsts.GlobalStyleSheet))
        {
            styleSheets.Add(EditorConsts.GlobalStyleSheet);
        }
    }

    public void Setup(SceneAsset sceneAsset)
    {
        this.sceneAsset = sceneAsset;

        scenePath = AssetDatabase.GetAssetPath(sceneAsset);

        SceneAsset.objectType = typeof(SceneAsset);
        SceneAsset.SetValueWithoutNotify(sceneAsset);
        SceneAsset.SetEnabled(false);

        bool isLoaded = SceneManager.GetSceneByPath(scenePath).isLoaded;
        bool isOnlyScene = SceneManager.loadedSceneCount == 1 && isLoaded;

        IsOpen.SetEnabled(false);
        IsOpen.SetValueWithoutNotify(isLoaded);

        LoadAdditivelyBtn.SetEnabled(!isLoaded);
        LoadSingleBtn.SetEnabled(!isOnlyScene);
        UnloadBtn.SetEnabled(isLoaded && !isOnlyScene);

        LoadAdditivelyBtn.clickable.clicked += HandleAdditive;
        LoadSingleBtn.clickable.clicked += HandleSingle;
        UnloadBtn.clickable.clicked += HandleUnload;
    }

    private void HandleSingle()
    {
        OnLoadSingle?.Invoke(this);
    }

    private void HandleUnload()
    {
        OnUnload?.Invoke(this);
    }

    private void HandleAdditive()
    {
        OnLoadAdditive?.Invoke(this);
    }
}

public class SceneLoadHelper : EditorWindow
{
    private const string USS_PATH = EditorConsts.GLOBAL_EDITOR_FOLRDER_PATH + "Scene Load Helper/SceneLoadHelper.uss";
    private const string UXML_PATH = EditorConsts.GLOBAL_EDITOR_FOLRDER_PATH + "Scene Load Helper/SceneLoadHelper.uxml";

    [MenuItem("Bloodthirst Tools/Scene Management/Scene Load Helper Window")]
    public static void ShowExample()
    {
        SceneLoadHelper wnd = GetWindow<SceneLoadHelper>();
        wnd.titleContent = new GUIContent("SceneLoadHelper");
    }

    VisualElement root;
    VisualElement RowsContainer => root.Q<VisualElement>(name = nameof(RowsContainer));
    Button LoadAllBtn => root.Q<Button>(name = nameof(LoadAllBtn));
    Button PlayFromEntryScene => root.Q<Button>(name = nameof(PlayFromEntryScene));

    private void CreateGUI()
    {
        EditorSceneManager.newSceneCreated += HandleSceneCreated;
        AssetWatcher.OnAssetEdited += HandleChange;
        AssetWatcher.OnAssetEdited += HandleChange;
        AssetWatcher.OnAssetCreated += HandleChange;
        AssetWatcher.OnAssetRemoved += HandleChange;
        AssetWatcher.OnAssetMoved += HandleChange;

        // Each editor window contains a root VisualElement object
        root = rootVisualElement;

        // Import UXML
        VisualTreeAsset visualTree = AssetDatabase.LoadAssetAtPath<VisualTreeAsset>(UXML_PATH);

        visualTree.CloneTree(root);

        StyleSheet styleSheet = AssetDatabase.LoadAssetAtPath<StyleSheet>(USS_PATH);

        root.styleSheets.Add(styleSheet);

        if (!root.styleSheets.Contains(EditorConsts.GlobalStyleSheet))
        {
            root.styleSheets.Add(EditorConsts.GlobalStyleSheet);
        }

        LoadAllBtn.clickable.clicked += HandleLoadAll;
        PlayFromEntryScene.clickable.clicked += HandlePlayFromEntry;

        RedrawUI();
    }

    private void OnDestroy()
    {
        EditorSceneManager.newSceneCreated -= HandleSceneCreated;
        AssetWatcher.OnAssetEdited -= HandleChange;
        AssetWatcher.OnAssetCreated -= HandleChange;
        AssetWatcher.OnAssetRemoved -= HandleChange;
        AssetWatcher.OnAssetMoved -= HandleChange;
    }

    private void HandlePlayFromEntry()
    {
        if (EditorApplication.isPlaying)
        {
            EditorApplication.ExitPlaymode();
        }

        string entryScenePath = "Assets/Scenes/EntryPoint/EntryPoint.unity";

        EditorSceneManager.OpenScene(entryScenePath, OpenSceneMode.Additive);

        foreach (SceneAsset s in GetScenes())
        {
            string path = AssetDatabase.GetAssetPath(s);

            Scene scene = EditorSceneManager.GetSceneByPath(path);

            if (scene.path == entryScenePath)
                continue;

            if (!scene.isLoaded)
                continue;

            EditorSceneManager.CloseScene(scene, true);
        }

        EditorApplication.EnterPlaymode();
    }

    private void HandleSceneCreated(Scene scene, NewSceneSetup setup, NewSceneMode mode)
    {
        EditorCoroutineUtility.StartCoroutine(CrtDelayedRefresh(), this);
    }

    private static void HandleChange(AssetImporter importer)
    {
        if (!HasOpenInstances<SceneLoadHelper>())
            return;

        SceneLoadHelper wnd = GetWindow<SceneLoadHelper>(nameof(SceneLoadHelper) , false);

        // if a folder is deleted, check if is have scene inside it
        if (AssetDatabase.IsValidFolder(importer.assetPath))
        {
            string[] results = AssetDatabase.FindAssets("t:scene", new string[] { importer.assetPath });

            if (results.Length != 0)
            {
                wnd.RedrawUI();
                return;
            }
        }

        if (!importer.assetPath.EndsWith(".unity"))
            return;

        EditorCoroutineUtility.StartCoroutine(wnd.CrtDelayedRefresh(), wnd);
    }

    private IEnumerator CrtDelayedRefresh()
    {
        double start = EditorApplication.timeSinceStartup;

        double diff = 0;
        while (diff < 0.25f)
        {
            diff = EditorApplication.timeSinceStartup - start;
            yield return null;
        }

        RedrawUI();
    }

    public void RedrawUI()
    {
        RowsContainer.Clear();

        // create a button for each scene
        foreach (SceneAsset s in GetScenes())
        {
            SceneEntry ui = new SceneEntry();

            RowsContainer.Add(ui);

            ui.Setup(s);

            ui.OnLoadAdditive += HandleAdditive;
            ui.OnLoadSingle += HandleSingle;
            ui.OnUnload += HandleUnload;
        }
    }

    private IEnumerable<SceneAsset> GetScenes()
    {
        return EditorUtils.FindAssetsByType<SceneAsset>("Assets/Scenes");
    }

    private void HandleLoadAll()
    {
        foreach (SceneAsset s in GetScenes())
        {
            string path = AssetDatabase.GetAssetPath(s);

            Scene scene = EditorSceneManager.GetSceneByPath(path);

            if (scene.isLoaded)
                continue;

            EditorSceneManager.OpenScene(path, OpenSceneMode.Additive);
        }

        RedrawUI();
    }

    private void HandleUnload(SceneEntry entry)
    {
        Scene scene = EditorSceneManager.GetSceneByPath(entry.ScenePath);
        EditorSceneManager.CloseScene(scene, true);

        RedrawUI();
    }

    private void HandleSingle(SceneEntry entry)
    {
        EditorSceneManager.OpenScene(entry.ScenePath, OpenSceneMode.Single);
        RedrawUI();
    }

    private void HandleAdditive(SceneEntry entry)
    {
        EditorSceneManager.OpenScene(entry.ScenePath, OpenSceneMode.Additive);
        RedrawUI();
    }
}