using Bloodthirst.Core.Utils;
using Bloodthirst.Editor;
using Bloodthirst.Editor.Commands;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using UnityEditor;
using UnityEditor.UIElements;
using UnityEngine.UIElements;

namespace Bloodthirst.Core.GameEventSystem
{
    public class CreateView : VisualElement
    {
        private const string UXML_PATH = "Packages/com.bloodthirst.bloodthirst-core/Editor/GameEventSystem/Editor/Main/CreateView/CreateView.uxml";
        private const string USS_PATH = "Packages/com.bloodthirst.bloodthirst-core/Editor/GameEventSystem/Editor/Main/CreateView/CreateView.uss";


        private TextField GameEventID => this.Q<TextField>(nameof(GameEventID));
        private TextField GameEventClass => this.Q<TextField>(nameof(GameEventClass));
        private Toggle AutoNamingToggle => this.Q<Toggle>(nameof(AutoNamingToggle));
        private Button AddEntryBtn => this.Q<Button>(nameof(AddEntryBtn));
        private Button RegenerateEnum => this.Q<Button>(nameof(RegenerateEnum));

        public GameEventSystemEditor Editor { get; set; }

        public CreateView()
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

        private void InitializeUI()
        {
            AutoNamingToggle.value = true;
            GameEventID.SetEnabled(false);
            AddEntryBtn.SetEnabled(false);
        }

        private void ListenUI()
        {
            GameEventID.RegisterValueChangedCallback(HandleDataChanged);
            GameEventClass.RegisterValueChangedCallback(HandleDataChanged);
            AutoNamingToggle.RegisterValueChangedCallback(HandleAutoNamingChanged);
            AddEntryBtn.clickable.clicked += HandleCreateClicked;
            RegenerateEnum.clickable.clicked += HandleRegenerateEnum;
        }

        private void HandleRegenerateEnum()
        {
            RefreshEnumScript();
        }

        private static IEnumerable<string> SplitOnCapitals(string text)
        {
            Regex regex = new Regex(@"\p{Lu}\p{Ll}*");
            foreach (Match match in regex.Matches(text))
            {
                yield return match.Value;
            }
        }

        private void HandleAutoNamingChanged(ChangeEvent<bool> evt)
        {
            GameEventID.SetEnabled(!evt.newValue);

            if (evt.newValue)
            {
                RefreshAutoName();
            }
        }

        private void RefreshAutoName()
        {
            string res = GenerateAutoName();

            GameEventID.value = res;
        }

        private string GenerateAutoName()
        {
            string res = string.Empty;
            string[] arr = SplitOnCapitals(GameEventClass.value).ToArray();

            if (arr.Length < 2)
            {
                return GameEventClass.value.ToUpper();
            }

            for (int i = 0; i < arr.Length - 1; i++)
            {
                string s = arr[i];
                res += s.ToUpper();
                res += "_";
            }

            res += arr[arr.Length - 1].ToUpper();
            return res;
        }

        private void HandleCreateClicked()
        {
            List<MonoScript> allScripts = EditorUtils.FindScriptAssets();
            MonoScript enumScript = allScripts
                .Where(m => m != null)
                .FirstOrDefault(m => m.name == Editor.GameEventAsset.enumName);

            if (enumScript == null)
            {
                throw new Exception("Couldn't find script");
            }

            Editor.GameEventAsset.Add(GameEventID.value, GameEventClass.value);

            CommandManagerEditor.RunInstant(new RegenerateEnumScriptCommand(Editor.GameEventAsset));

            CommandManagerEditor.RunInstant(new CreateGameEventClassType
                (
                Editor.GameEventAsset,
                GameEventID.value,
                GameEventClass.value
                ));
        }

        private void RefreshEnumScript()
        {
            CommandManagerEditor.RunInstant(new RegenerateEnumScriptCommand(Editor.GameEventAsset));
        }

        private void HandleDataChanged(ChangeEvent<string> evt)
        {
            Refresh();
        }

        private bool CanCreate()
        {
            if (string.IsNullOrWhiteSpace(GameEventClass.value))
                return false;

            if (string.IsNullOrWhiteSpace(GameEventID.value))
                return false;

            return true;
        }

        private void Refresh()
        {
            if (AutoNamingToggle.value)
            {
                RefreshAutoName();
            }

            bool val = CanCreate();


            AddEntryBtn.SetEnabled(val);
        }


    }
}
