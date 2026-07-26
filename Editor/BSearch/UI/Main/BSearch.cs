using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;
using UnityEditor.UIElements;
using Bloodthirst.Editor.BInspector;
using System;
using System.Collections.Generic;
using Bloodthirst.Core.Utils;
using System.Linq;
using System.Reflection;
using System.Text;
using UnityEngine.SceneManagement;
using UnityEditor.SceneManagement;
using UnityEngine.Pool;
using UnityEditor.SceneTemplate;

namespace Bloodthirst.Editor.BSearch
{
    public class BSearch : EditorWindow
    {
        private const string UXML_PATH = "Packages/com.bloodthirst.bloodthirst-core/Editor/BSearch/UI/Main/BSearch.uxml";
        private const string USS_PATH = "Packages/com.bloodthirst.bloodthirst-core/Editor/BSearch/UI/Main/BSearch.uss";

        [MenuItem("Bloodthirst Tools/BSearch")]
        public static void ShowExample()
        {
            BSearch wnd = GetWindow<BSearch>();
            wnd.titleContent = new GUIContent("BSearch");
        }

        private IBSearchFilter currentSearchFilter;

        private List<BSearchResultPath> CachedUIs { get; set; } = new List<BSearchResultPath>();

        private IBSearchFilter CurrentSearchFilter
        {
            get => currentSearchFilter;
            set
            {
                if (currentSearchFilter != null)
                {
                    currentSearchFilter.OnFilterChanged -= HandleFilterChanged;
                }

                currentSearchFilter = value;
                currentSearchFilter.OnFilterChanged -= HandleFilterChanged;
                currentSearchFilter.OnFilterChanged += HandleFilterChanged;

                InitFilterSettingsUI();
            }
        }


        private VisualElement Root { get; set; }

        public VisualElement FilterDropdownContainer => Root.Q<VisualElement>((nameof(FilterDropdownContainer)));

        private PopupField<IBSearchFilter> FilterDropdown { get; set; }

        private VisualElement FilterSettings => Root.Q<VisualElement>(nameof(FilterSettings));

        private VisualElement SearchSection => Root.Q<VisualElement>(nameof(SearchSection));

        private Toggle Prefabs => Root.Q<Toggle>(nameof(Prefabs));
        private Toggle ScriptableObjects => Root.Q<Toggle>(nameof(ScriptableObjects));
        private Toggle Scenes => Root.Q<Toggle>(nameof(Scenes));

        private Button SearchBtn => Root.Q<Button>(nameof(SearchBtn));

        private VisualElement SearchResultContainer => Root.Q<VisualElement>(nameof(SearchResultContainer));

        public void OnEnable()
        {
            EditorSceneManager.sceneOpened -= HandleSceneOpened;
            EditorSceneManager.sceneOpened += HandleSceneOpened;
        }

        public void OnDisable()
        {
            EditorSceneManager.sceneOpened -= HandleSceneOpened;
        }

        private void HandleSceneOpened(Scene scene, OpenSceneMode mode)
        {
            for (int i = 0; i < CachedUIs.Count; i++)
            {
                BSearchResultPath ui = CachedUIs[i];
                ui.Refresh();
            }
        }

        public void CreateGUI()
        {
            // root
            VisualElement root = rootVisualElement;

            root.Clear();

            // uxml
            var visualTree = AssetDatabase.LoadAssetAtPath<VisualTreeAsset>(UXML_PATH);
            visualTree.CloneTree(root);

            // stylesheet
            StyleSheet styleSheet = AssetDatabase.LoadAssetAtPath<StyleSheet>(USS_PATH);

            root.styleSheets.Add(styleSheet);
            root.styleSheets.Add(EditorConsts.GlobalStyleSheet);

            Root = root;

            InitilizeUI();
        }

        private string FormatString(IBSearchFilter searchFilter)
        {
            BSearchFilterNameAttribute attr = (BSearchFilterNameAttribute)Attribute.GetCustomAttribute(searchFilter.GetType(), typeof(BSearchFilterNameAttribute));
            return attr.Name;
        }


        private void HandleSelectionTypeChanged(ChangeEvent<IBSearchFilter> evt)
        {
            CurrentSearchFilter = evt.newValue;
        }

        private void HandleFilterChanged(IBSearchFilter filter)
        {
            RefreshSearch();
        }

        private void RefreshSearch()
        {

            if (currentSearchFilter == null)
            {
                SearchSection.SetEnabled(false);
                return;
            }

            bool isValid = currentSearchFilter.IsValid();

            SearchSection.SetEnabled(isValid);
        }

        private void InitFilterSettingsUI()
        {
            IBInspectorDrawer drawer = new BInspectorDefault();
            VisualElement ui = drawer.CreateInspectorGUI(CurrentSearchFilter).RootContainer;

            FilterSettings.Clear();
            FilterSettings.Add(ui);
            RefreshSearch();
            RefreshButton();
        }

        private void InitilizeUI()
        {
            List<IBSearchFilter> filters = TypeUtils.AllTypes
                .Where(t => t.IsClass)
                .Where(t => !t.IsAbstract)
                .Where(t => TypeUtils.IsSubTypeOf(t, typeof(IBSearchFilter)))
                .Select(t => (IBSearchFilter)Activator.CreateInstance(t))
                .ToList();

            // selection
            FilterDropdown = new PopupField<IBSearchFilter>("Filter", filters, 0, FormatString, FormatString);
            FilterDropdown.UnregisterValueChangedCallback(HandleSelectionTypeChanged);
            FilterDropdown.RegisterValueChangedCallback(HandleSelectionTypeChanged);

            FilterDropdownContainer.Add(FilterDropdown);

            // search target
            ScriptableObjects.RegisterValueChangedCallback(HandleSearchTargetChanged);
            Prefabs.RegisterValueChangedCallback(HandleSearchTargetChanged);
            Scenes.RegisterValueChangedCallback(HandleSearchTargetChanged);

            // button
            SearchBtn.clickable.clicked -= HandleSeachBtnClicked;
            SearchBtn.clickable.clicked += HandleSeachBtnClicked;

            // apply state
            CurrentSearchFilter = FilterDropdown.value;
            SearchResultContainer.Display(false);


        }

        private void HandleSearchTargetChanged(ChangeEvent<bool> evt)
        {
            RefreshButton();
        }

        private void RefreshButton()
        {
            bool hasSearchTargetSelected = Prefabs.value || Scenes.value || ScriptableObjects.value;

            SearchBtn.SetEnabled(hasSearchTargetSelected);
        }

        private static List<Type> OrderByType { get; } = new List<Type>()
        {
            typeof(ScriptableObject),
            typeof(GameObject),
            typeof(SceneAsset)
        };



        private void HandleSeachBtnClicked()
        {
            SearchResultContainer.Clear();
            CachedUIs.Clear();

            List<List<ResultPath>> results = new List<List<ResultPath>>();

            if (ScriptableObjects.value)
            {
                List<object> objs = EditorUtils.FindAssetsByType<ScriptableObject>().Where(s => !(s is SceneTemplateAsset)).Cast<object>().ToList();
                List<List<ResultPath>> res = CurrentSearchFilter.GetSearchResults(objs);

                results.AddRange(res);
            }

            if (Prefabs.value)
            {
                IEnumerable<object> objs = EditorUtils.FindAssetsAs<GameObject>("t:prefab").Cast<object>();
                List<List<ResultPath>> res = CurrentSearchFilter.GetSearchResults(objs);

                results.AddRange(res);
            }

            if (Scenes.value)
            {
                // save previously open scenes
                using (ListPool<Scene>.Get(out List<Scene> prevSceneDetails))
                {
                    SceneUtils.GetAllScenesInHierarchy(out Scene active, prevSceneDetails);

                    // search
                    IEnumerable<SceneAsset> allScenes = EditorUtils.FindAssetsAs<SceneAsset>("t:scene");
                    List<object> objs = allScenes.Cast<object>().ToList();

                    // this already opens all the scenes
                    List<List<ResultPath>> res = CurrentSearchFilter.GetSearchResults(objs);
                    results.AddRange(res);

                    // restore the scenes
                    IEnumerable<string> scenesToUnload = allScenes.Select(s => AssetDatabase.GetAssetPath(s)).Except(prevSceneDetails.Select(s => s.path));

                    foreach (string s in scenesToUnload)
                    {
                        Scene scene = SceneManager.GetSceneByPath(s);

                        if (scene.isLoaded)
                        {
                            EditorSceneManager.CloseScene(scene, true);
                        }
                    }

                    SceneManager.SetActiveScene(active);
                }
            }


            if (results.Count == 0)
            {
                SearchResultContainer.Display(false);
                return;
            }

            SearchResultContainer.Display(true);

            foreach (List<ResultPath> resultPath in results)
            {
                BSearchResult r = new BSearchResult();

                for (int i = 0; i < resultPath.Count; i++)
                {
                    ResultPath p = resultPath[i];
                    BSearchResultPath path = new BSearchResultPath();
                    path.Setup(p, resultPath.Count - i);

                    r.PathsContainer.Add(path);
                    CachedUIs.Add(path);
                }

                SearchResultContainer.Add(r);
            }

        }

        private void Clear()
        {
            // selection type
            if (FilterDropdown != null)
            {
                FilterDropdown.UnregisterValueChangedCallback(HandleSelectionTypeChanged);
                FilterDropdownContainer.Remove(FilterDropdown);
                FilterDropdown = null;
            }

            // search target
            ScriptableObjects.UnregisterValueChangedCallback(HandleSearchTargetChanged);
            Prefabs.UnregisterValueChangedCallback(HandleSearchTargetChanged);
            Scenes.UnregisterValueChangedCallback(HandleSearchTargetChanged);

            currentSearchFilter = null;

            // button
            SearchBtn.clickable.clicked -= HandleSeachBtnClicked;

        }

        private void OnDestroy()
        {
            Clear();
        }
    }
}