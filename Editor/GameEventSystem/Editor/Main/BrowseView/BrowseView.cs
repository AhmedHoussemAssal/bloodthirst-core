using Bloodthirst.Editor;
using Bloodthirst.Editor.CustomComponent;
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine.UIElements;

namespace Bloodthirst.Core.GameEventSystem
{
    public class BrowseView : VisualElement
    {
        private const string UXML_PATH = "Packages/com.bloodthirst.bloodthirst-core/Editor/GameEventSystem/Editor/Main/BrowseView/BrowseView.uxml";
        private const string USS_PATH = "Packages/com.bloodthirst.bloodthirst-core/Editor/GameEventSystem/Editor/Main/BrowseView/BrowseView.uss";

        public GameEventSystemEditor Editor { get; set; }
        private SearchableList SearchableList => this.Q<SearchableList>(nameof(SearchableList));
        public List<GameEventSystemAsset.EnumClassPair> GetAll { get; private set; }
        
        public BrowseView()
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
        }

        public void Setup(GameEventSystemEditor editor)
        {
            Editor = editor;
            GetAll = Editor.GameEventAsset.GetAll().ToList();
            SearchableList.Setup(GetAll, FilterCondition , MakeElement, BindElement , null);
        }

        private void BindElement(VisualElement visualElement, int index)
        {
            BrowseElement casted = (BrowseElement)visualElement;

            IndexWrapper actualVal = SearchableList.CurrentValues[index];

            casted.Setup(Editor, actualVal);
        }
        private VisualElement MakeElement()
        {
            return new BrowseElement();
        }
        private bool FilterCondition(IndexWrapper val , string searchTerm)
        {
            GameEventSystemAsset.EnumClassPair casted = (GameEventSystemAsset.EnumClassPair)val.Value;

            return (casted.className.ToLowerInvariant() + casted.enumValue.ToLowerInvariant()).ToLowerInvariant()
                                                                        .Contains(searchTerm.ToLowerInvariant());
        }

        internal void Refresh()
        {
            SearchableList.RefreshResults();
        }
    }
}
