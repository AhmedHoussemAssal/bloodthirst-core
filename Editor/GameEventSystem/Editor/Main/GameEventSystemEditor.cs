using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;
using Bloodthirst.Editor;
using System.Collections.Generic;
using System.Linq;
using System;
using Bloodthirst.Core.Utils;
using Bloodthirst.Editor.CustomComponent;

namespace Bloodthirst.Core.GameEventSystem
{
    public class GameEventSystemEditor : EditorWindow
    {
        private const string UXML_PATH = "Packages/com.bloodthirst.bloodthirst-core/Editor/GameEventSystem/Editor/Main/GameEventSystemEditor.uxml";
        private const string USS_PATH = "Packages/com.bloodthirst.bloodthirst-core/Editor/GameEventSystem/Editor/Main/GameEventSystemEditor.uss";     

        [MenuItem("Bloodthirst Tools/GameEventSystem")]
        public static void ShowExample()
        {
            GameEventSystemEditor wnd = GetWindow<GameEventSystemEditor>();
            wnd.titleContent = new GUIContent("GameEventSystemEditor");
        }

        private VisualElement root;

        private TwoPaneSplitView PanelSplit => root.Q<TwoPaneSplitView>(nameof(PanelSplit));


        internal SearchableDropdown DropdownUI { get; set; }
        private VisualElement DropdownContainer => root.Q<VisualElement>(nameof(DropdownContainer));
        private Button CreateNew => root.Q<Button>(nameof(CreateNew));
        private VisualElement MainContent => root.Q<VisualElement>(nameof(MainContent));
        public TabUI Tabs => root.Q<TabUI>(nameof(Tabs));
        public TabElement ManageTab => root.Q<TabElement>(nameof(ManageTab));
        public TabElement EditTab => root.Q<TabElement>(nameof(EditTab));
        public TabElement CheckTab => root.Q<TabElement>(nameof(CheckTab));
        private CreateView CreateView => root.Q<CreateView>(nameof(CreateView));
        private BrowseView BrowseView => root.Q<BrowseView>(nameof(BrowseView));
        public VisualElement EditTabContainer { get; private set; }
        public VisualElement DeleteTabContainer { get; private set; }

        private event Action<GameEventSystemAsset> OnGameEventAssetChanged;

        private GameEventSystemAsset gameEventAsset;
        internal GameEventSystemAsset GameEventAsset
        {
            get => gameEventAsset;
            set
            {
                if (gameEventAsset == value)
                    return;

                gameEventAsset = value;
                OnGameEventAssetChanged?.Invoke(gameEventAsset);
            }
        }

        public void CreateGUI()
        {
            // Each editor window contains a root VisualElement object
            root = rootVisualElement;

            // import USS
            StyleSheet styleSheet = AssetDatabase.LoadAssetAtPath<StyleSheet>(USS_PATH);

            // Import UXML
            VisualTreeAsset visualTree = AssetDatabase.LoadAssetAtPath<VisualTreeAsset>(UXML_PATH);
            visualTree.CloneTree(root);

            root.styleSheets.Add(styleSheet);
            if (!root.styleSheets.Contains(EditorConsts.GlobalStyleSheet))
            {
                root.styleSheets.Add(EditorConsts.GlobalStyleSheet);
            }
            InitializeUI();

            ListenEvents();
        }

        private VisualElement MakeItem()
        {
            return new SearchableDropdownAssetElement();
        }
        
        private void BindItem(VisualElement element, int index)
        {
            SearchableDropdownAssetElement casted = (SearchableDropdownAssetElement)element;

            IndexWrapper wrapped = DropdownUI.CurrentDropdown.SearchableList.CurrentValues[index];

            casted.Setup(wrapped);
        }
        
        private bool SearchTerm(IndexWrapper item , string searchTerm)
        {
            UnityEngine.Object casted = (UnityEngine.Object)item.Value;

            return casted.name.ToLowerInvariant().Contains(searchTerm.ToLowerInvariant());
        }

        private string SelectedAsString(object item)
        {
            UnityEngine.Object casted = (UnityEngine.Object)((IndexWrapper)item).Value;

            return casted.name;
        }

        internal void UpdateDropdownOptions()
        {
            List<GameEventSystemAsset> allAssets = GetAllGameEventAssets();
            DropdownUI.UpdateSource(allAssets);
        }

        private List<GameEventSystemAsset> GetAllGameEventAssets()
        {
            List<GameEventSystemAsset> assets = AssetDatabase.FindAssets("t:GameEventSystemAsset")
               .Select(guid => AssetDatabase.LoadAssetByGUID<GameEventSystemAsset>(new GUID(guid)))
               .ToList();

            return assets;
        }

        private void InitializeUI()
        {
            List<GameEventSystemAsset> allAssets = GetAllGameEventAssets();

            DropdownUI = new SearchableDropdown("Select Datasource", -1, allAssets, this, MakeItem, BindItem, SearchTerm, SelectedAsString);
            DropdownContainer.Add(DropdownUI);

            PanelSplit.fixedPaneIndex = 1;
            PanelSplit.fixedPaneInitialDimension = 250;

            RefreshTabs();

            BrowseView.Editor = this;
            CreateView.Editor = this;
        }

        private void RefreshTabs()
        {
            Tabs.Select(0);

            if (gameEventAsset == null)
            {
                Tabs.Display(false);
                return;
            }

            Tabs.Display(true);

            BrowseView.Setup(this);           
        }

        private void HandleCreateNewGameEventSystem()
        {
            CreatePopup wnd = CreateInstance<CreatePopup>();
            wnd.titleContent = new GUIContent("Create new GameEventSystem");
            wnd.FromWindow = this;
            wnd.ShowModal();
        }

        private void ListenEvents()
        {
            DropdownUI.OnValueChanged -= HandleDropdownAssetChanged;
            DropdownUI.OnValueChanged += HandleDropdownAssetChanged;

            CreateNew.clickable.clicked -= HandleCreateNewGameEventSystem;
            CreateNew.clickable.clicked += HandleCreateNewGameEventSystem;

            OnGameEventAssetChanged -= HandleAssetChanged;
            OnGameEventAssetChanged += HandleAssetChanged;

            AssemblyReloadEvents.afterAssemblyReload += ReloadList;
        }

        private static void ReloadList()
        {
            if (!HasOpenInstances<GameEventSystemEditor>())
                return;

            GameEventSystemEditor wnd = GetWindow<GameEventSystemEditor>();
            wnd.BrowseView.Refresh();
        }

        private void HandleDropdownAssetChanged(IndexWrapper dropdownItem)
        {
            GameEventAsset = (GameEventSystemAsset)dropdownItem.Value;
        }

        private void HandleAssetChanged(GameEventSystemAsset val)
        {
            RefreshTabs();
        }

        private void UnlistenEvents()
        {
            DropdownUI.OnValueChanged -= HandleDropdownAssetChanged;
            CreateNew.clickable.clicked -= HandleCreateNewGameEventSystem;
            OnGameEventAssetChanged -= HandleAssetChanged;
            AssemblyReloadEvents.afterAssemblyReload -= ReloadList;
        }


        private void OnDestroy()
        {
            UnlistenEvents();
        }
    }
}