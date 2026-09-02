using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.UIElements;
using UnityEngine.UIElements;

namespace Bloodthirst.Editor.CustomComponent
{
    public class SearchableList : VisualElement
    {

        private const string UXML_PATH = "Packages/com.bloodthirst.bloodthirst-core/Editor/CustomUIToolkit.Editor/SearchableList/SearchableList.uxml";
        private const string USS_PATH = "Packages/com.bloodthirst.bloodthirst-core/Editor/CustomUIToolkit.Editor/SearchableList/SearchableList.uss";

        private ToolbarSearchField SearchBar => this.Q<ToolbarSearchField>(nameof(SearchBar));
        public ListView SearchResult => this.Q<ListView>(nameof(SearchResult));

        private List<IndexWrapper> _AllValues;
        private List<IndexWrapper> _FilteredValues;
        public IReadOnlyList<IndexWrapper> CurrentValues => _FilteredValues;
        
        private Func<IndexWrapper, string, bool> _FilterCondition;
        private Func<VisualElement> _MakeItem;
        private Action<VisualElement, int> _BindItem;
        private Action<IEnumerable<object>> _OnSelectionChanged;

        public SearchableList()
        {
            // import USS
            StyleSheet styleSheet = AssetDatabase.LoadAssetAtPath<StyleSheet>(USS_PATH);

            // Import UXML
            VisualTreeAsset visualTree = AssetDatabase.LoadAssetAtPath<VisualTreeAsset>(UXML_PATH);
            visualTree.CloneTree(this);

            styleSheets.Add(styleSheet);
            if (!styleSheets.Contains(EditorConsts.GlobalStyleSheet))
            {
                styleSheets.Add(EditorConsts.GlobalStyleSheet);
            }

            InitializeUI();
            ListenUI();
        }

        public void Setup(IList source, Func<IndexWrapper, string, bool> _filterCondition, Func<VisualElement> _MakeItem, Action<VisualElement, int> _BindItem, Action<IEnumerable<object>> _OnSelectionChanged)
        {
            _FilteredValues = new List<IndexWrapper>(source.Count);
            _AllValues = new List<IndexWrapper>(source.Count);

            for(int i = 0; i < source.Count; i++)
            {
                IndexWrapper indexed = new IndexWrapper() { Index = i, Value = source[i] };
                _FilteredValues.Add(indexed);
                _AllValues.Add(indexed);
            }

            this._FilterCondition = _filterCondition;
            this._MakeItem = _MakeItem;
            this._BindItem = _BindItem;
            this._OnSelectionChanged = _OnSelectionChanged;

            SearchResult.itemsSource = _FilteredValues;

            SearchResult.makeItem = this._MakeItem;
            SearchResult.bindItem = this._BindItem;
            
            SearchResult.selectionChanged -= this._OnSelectionChanged;
            SearchResult.selectionChanged += this._OnSelectionChanged;

            SearchResult.RefreshItems();

        }

        private void InitializeUI()
        {
            SearchResult.virtualizationMethod = CollectionVirtualizationMethod.DynamicHeight;
            SearchResult.horizontalScrollingEnabled = false;
        }

        private void ListenUI()
        {
            SearchBar.RegisterValueChangedCallback(HandleSearchValueChanged);
        }

        private void HandleSearchValueChanged(ChangeEvent<string> evt)
        {
            RefreshResults();
        }

        public void RefreshResults()
        {
            string searchTerm = SearchBar.value;

            _FilteredValues.Clear();

            if (string.IsNullOrWhiteSpace(searchTerm))
            {
                for (int i = 0; i < _AllValues.Count; i++)
                {
                    IndexWrapper v = _AllValues[i];
                    _FilteredValues.Add(v);
                }
            }
            else
            {
                for (int i = 0; i < _AllValues.Count; i++)
                {
                    IndexWrapper curItem = _AllValues[i];

                    if (_FilterCondition(curItem , searchTerm))
                    {
                        _FilteredValues.Add(curItem);
                    }
                }
            }

            SearchResult.RefreshItems();
        }


    }
}
