using Bloodthirst.Core.Utils;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine.UIElements;

namespace Bloodthirst.Editor.CustomComponent
{
    public class TabUI : VisualElement
    {
        private const string UXML_PATH = "Packages/com.bloodthirst.bloodthirst-core/Editor/CustomUIToolkit.Editor/TabUI/TabUI.uxml";
        private const string USS_PATH = "Packages/com.bloodthirst.bloodthirst-core/Editor/CustomUIToolkit.Editor/TabUI/TabUI.uss";

        public int CurrentTabIndex { get; private set; }

        public override VisualElement contentContainer => TabContainer;

        private List<TabButton> AllTabButtons { get; set; } = new List<TabButton>();
        private List<TabContent> AllTabContents { get; set; } = new List<TabContent>();
        private VisualElement TabContainer => this.Q<VisualElement>(nameof(TabContainer));
        private VisualElement ContentContainer => this.Q<VisualElement>(nameof(ContentContainer));

        public TabUI()
        {
            SetupElement();
            InitializeUI();
        }

        private void SetupElement()
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

        public void Select(int tabIndex)
        {
            CurrentTabIndex = tabIndex;

            for (int i = 0; i < AllTabContents.Count; i++)
            {
                TabContent c = AllTabContents[i];
                c.Display( i == tabIndex);
            }

            for (int i = 0; i < AllTabButtons.Count; i++)
            {
                TabButton curr = AllTabButtons[i];
                curr.UnSelected();
            }

            AllTabButtons[tabIndex].Select();
        }

        private void InitializeUI()
        {
            CheckForTabs();
            TabContainer.RegisterCallback<GeometryChangedEvent>(HandleContentChanged);
            TabContainer.RegisterCallback<AttachToPanelEvent>(HandleAttachedToPanel);
        }

        private void HandleAttachedToPanel(AttachToPanelEvent evt)
        {
            CheckForTabs();
        }

        private void HandleContentChanged(GeometryChangedEvent evt)
        {
            CheckForTabs();
        }

        public void CheckForTabs()
        {
            List<TabButton> newTabButtons = new List<TabButton>();
            List<TabContent> newTabContents = new List<TabContent>();
            int index = 0;

            int i = 0;

            // the goal of this loop is to check if we have any "TabElement" children that haven't been converted to a "TabButton"-"TabContent" pair
            // if we find any , we convert them into a TabButton and TabContent and we remove the TabElement since it's now "Setup" correctly
            while (i < contentContainer.childCount)
            {
                VisualElement curr = contentContainer[i];

                // if we found an "uncoverted" tab
                // we do the setup
                if (curr is TabElement t)
                {
                    // create button
                    TabButton newTabButton = new TabButton(this, t.Title, index);

                    // copy content
                    TabContent newTabContent = new TabContent(this, index);

                    // we take the content form the TabElement and we move it to the TabContent
                    while(t.contentContainer.childCount != 0)
                    {
                        VisualElement tabC = t.contentContainer[0];

                        // NOTE : apprentyl the "Add" method already removes the element from the previous parent
                        // so no need to do something like : oldParent.Remove(child) ; newParent.Add(child)
                        // we just add directly
                        newTabContent.Add(tabC);        
                    }

                    newTabButtons.Add(newTabButton);
                    newTabContents.Add(newTabContent);

                    index++;

                    contentContainer.RemoveAt(i);
                }

                else
                {
                    i++;
                }
            }


            // we add all the buttons to the "button" header
            // and to a buttons collection 
            foreach (TabButton b in newTabButtons)
            {
                AllTabButtons.Add(b);
                TabContainer.Add(b);
            }

            // add to a contents collection
            // then the THIS tabUI will decide which tab content to show
            foreach (TabContent c in newTabContents)
            {
                AllTabContents.Add(c);
                ContentContainer.Add(c);
            }

            
 
        }
    }
}
