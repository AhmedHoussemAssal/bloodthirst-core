using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;
using System.Collections.Generic;
using System;
using UnityEditor.UIElements;
using Bloodthirst.Core.Utils;
using System.Linq;
using UnityEditor.Callbacks;
using Unity.EditorCoroutines.Editor;
using System.Collections;
#if ODIN_INSPECTOR
using Sirenix.Utilities;
#endif
using Bloodthirst.Runtime.BNodeTree;
using Bloodthirst.BEventSystem;
using UnityEngine.Assertions;
using System.IO;

namespace Bloodthirst.Editor.BNodeTree
{
    public class BNodeTreeEditor : EditorWindow, INodeEditor
    {
        private static NodeTreeData RequestOpen { get; set; }

        /// <summary>
        /// UXML path
        /// </summary>
        private const string UXML_PATH = BNodeTreeEditorUtils.EDITOR_BASE_PATH + "/BNodeTreeEditor.uxml";

        /// <summary>
        /// USS path
        /// </summary>
        private const string USS_PATH = BNodeTreeEditorUtils.EDITOR_BASE_PATH + "/BNodeTreeEditor.uss";

        private const int MAX_NODE_ID = 100;
        private float zoom = 1;


        [MenuItem("Bloodthirst Tools/BNodeTreeEditor")]
        public static void OpenWindow()
        {
            BNodeTreeEditor wnd = GetWindow<BNodeTreeEditor>();
            wnd.titleContent = new GUIContent("BNodeTreeEditor");
        }

        /*
        [DidReloadScripts()]
        public static void OnScriptReload()
        {
            if (!EditorWindow.HasOpenInstances<BQuestSystemEditor>())
                return;

            BQuestSystemEditor win = GetWindow<BQuestSystemEditor>();

            if (!win.IsInitialized)
                return;

            win.RefreshObjectPickers(true);

        }
        */

        #region global params
        public bool IsInitialized { get; set; }
        public float ZoomSensitivity { get; set; } = 0.01f;
        public bool InvertZoom { get; set; } = true;
        public Vector2 ZoomMinMax { get; set; }
        public Type NodeBaseType { get; set; }
        public float Zoom
        {
            get => zoom;
            set
            {
                float old = zoom;
                zoom = Mathf.Clamp(value, ZoomMinMax.x, ZoomMinMax.y);
                BEventSystem.Trigger(new OnZoomChanged(this, old, zoom));
            }
        }

        public Vector2 PanningOffset { get; set; }

        #endregion

        #region UI elements

        private VisualElement Root => rootVisualElement;
        private VisualElement NodeTypePickerContainer => Root.Q<VisualElement>(nameof(NodeTypePickerContainer));
        private PopupField<Type> NodeTypePicker { get; set; }
        public VisualElement Canvas => Root.Q<VisualElement>(nameof(Canvas));
        public VisualElement Container => Root.Q<VisualElement>(nameof(Container));
        public Button SaveBtn => Root.Q<Button>(nameof(SaveBtn));
        public Button LoadBtn => Root.Q<Button>(nameof(LoadBtn));
        public ObjectField NodeTreeDataPicker => Root.Q<ObjectField>(nameof(NodeTreeDataPicker));
        public ObjectField NodeBehaviourPicker => Root.Q<ObjectField>(nameof(NodeBehaviourPicker));

        #endregion

        #region nodes state

        private List<int> CurrentNodeIDs { get; set; }
        private HashSet<NodeBaseElement> SelectedNodes { get; set; }
        private List<NodeBaseElement> AllNodes { get; set; }
        private List<LinkElement> AllLinks { get; set; }

        private IBNodeTreeBehaviour selectedTreeBehaviour;
        private IBNodeTreeBehaviour SelectedTreeBehaviour
        {
            get => selectedTreeBehaviour;
            set
            {
                EditorCoroutineUtility.StartCoroutine(CrtSetSelectedTreeBehaviour(value), this);
            }
        }



        private IBNodeTreeBehaviour SelectedTreeBehaviourForce
        {
            set
            {
                EditorCoroutineUtility.StartCoroutine(CrtSetSelectedTreeBehaviourForce(value), this);
            }
        }

        private NodeTreeData selectedNodeData;


        private NodeTreeData SelectedTreeData
        {
            get => selectedNodeData;
            set
            {
                EditorCoroutineUtility.StartCoroutine(CrtSetSelectedTreeData(value), this);

            }
        }

        private NodeTreeData SelectedTreeDataForce
        {
            set
            {
                EditorCoroutineUtility.StartCoroutine(CrtSetSelectedTreeDataForce(value), this);
            }
        }
        #endregion

        #region crt setters
        private IEnumerator CrtSetSelectedTreeDataForce(NodeTreeData value)
        {
            // refresh ui first
            NodeTreeDataPicker.value = value;

            selectedNodeData = value;

            if (selectedNodeData != null)
            {
                yield return CrtLoad(selectedNodeData);
            }

            yield return CrtTriggerSelectTreeDataChanged();
        }

        private IEnumerator CrtSetSelectedTreeData(NodeTreeData value)
        {
            // refresh ui first
            NodeTreeDataPicker.value = value;

            if (selectedNodeData == value)
                yield break;

            selectedNodeData = value;

            if (selectedNodeData != null)
            {
                yield return CrtLoad(selectedNodeData);
            }

            yield return CrtTriggerSelectTreeDataChanged();
        }

        private IEnumerator CrtTriggerSelectTreeDataChanged()
        {
            BEventSystem.Trigger(new OnDataSelectionChanged(this, selectedNodeData));
            yield break;
        }

        private IEnumerator CrtSetSelectedTreeBehaviourForce(IBNodeTreeBehaviour value)
        {
            // refresh ui first
            NodeBehaviourPicker.value = (UnityEngine.Object)value;

            selectedTreeBehaviour = value;

            // change the selected data

            if (selectedTreeBehaviour != null)
            {
                yield return CrtSetSelectedTreeDataForce(selectedTreeBehaviour.TreeData);
            }

            // wait for total nodes execution
            yield return CrtTriggerSelectTreeBehaviourChanged();
        }

        private IEnumerator CrtSetSelectedTreeBehaviour(IBNodeTreeBehaviour value)
        {
            // refresh ui first
            NodeBehaviourPicker.value = (UnityEngine.Object)value;

            if (selectedTreeBehaviour == value)
            {
                yield break;
            }

            selectedTreeBehaviour = value;

            // change the selected data

            if (selectedTreeBehaviour != null)
            {
                yield return CrtSetSelectedTreeData(selectedTreeBehaviour.TreeData);
            }

            // wait for total nodes execution
            yield return CrtTriggerSelectTreeBehaviourChanged();
        }


        private IEnumerator CrtTriggerSelectTreeBehaviourChanged()
        {
            BEventSystem.Trigger(new OnBehaviourSelectionChanged(this, selectedTreeBehaviour));
            yield break;
        }
        #endregion

        Type INodeEditor.NodeBaseType => NodeBaseType;
        HashSet<NodeBaseElement> INodeEditor.SelectedNodes => SelectedNodes;
        List<NodeBaseElement> INodeEditor.AllNodes => AllNodes;
        List<LinkElement> INodeEditor.AllLinks => AllLinks;
        List<int> INodeEditor.CurrentNodeIDs => CurrentNodeIDs;

        public BEventSystem<BNodeTreeEventBase> BEventSystem { get; set; }

        private List<INodeEditorAction> CanvasActions { get; set; }

        public Vector2 CanvasSize => Container.localBound.size;

        private void OnDisable()
        {
            if (!IsInitialized)
                return;

            foreach (INodeEditorAction a in CanvasActions)
            {
                a.OnDisable();
                a.NodeEditor = null;
            }

            SaveBtn.clickable.clicked -= HandleSaveClicked;
            LoadBtn.clickable.clicked -= HandleLoadClicked;
            Root.RegisterCallback<KeyDownEvent>(OnKeyPress);

            //// Register container events
            // scroll
            Container.UnregisterCallback<ClickEvent>(OnMouseClick);
            Container.UnregisterCallback<ContextClickEvent>(OnRightClick);
            Container.UnregisterCallback<WheelEvent>(OnScrollWheel);
            Container.UnregisterCallback<MouseDownEvent>(OnMouseDown);
            Container.UnregisterCallback<MouseLeaveEvent>(OnMouseLeave);
            Container.UnregisterCallback<MouseUpEvent>(OnMouseUp);
            Container.UnregisterCallback<MouseMoveEvent>(OnMouseMove);
            NodeTypePicker.UnregisterValueChangedCallback(HandleNodeTypeChanged);


            EditorApplication.update -= Update;
            EditorApplication.playModeStateChanged -= HandlePlayModeStateChanged;
            Selection.selectionChanged -= HandleObjectSelection;

            IsInitialized = false;
        }

        private void HandleObjectSelection()
        {
            IBNodeTreeBehaviour tree = TryGetSelectedTreeBehaviour();
            if (tree != null)
            {
                SelectedTreeBehaviour = tree;
                return;
            }

            NodeTreeData data = TryGetSelectedTreeData();

            if (data != null)
            {
                SelectedTreeData = data;
            }
        }

        private void ClearCanvas()
        {
            for (int i = AllNodes.Count - 1; i >= 0; i--)
            {
                NodeBaseElement curr = AllNodes[i];
                RemoveNode(curr.NodeType);
            }
        }

        private void Update()
        {
            if (Canvas == null)
                return;

            Canvas.transform.scale = Vector3.one * Zoom;
            Canvas.transform.position = PanningOffset * Zoom;
        }

        private void SetupCanvasActions()
        {
            CanvasActions = new List<INodeEditorAction>()
            {
                // nodes
                new AddNodeAction(),
                new RemoveNodeAction(),
                new ResizeNodeAction(),
                new AddPortInputAction(),
                new AddPortOutputAction(),
                new RemovePortAction(),
                new UnlinkNodeAction(),
                new TogglePortInfoAction(),

                // selection
                new SelectNodeAction(),
                new DeselectNodeAction(),

                // link
                new LinkNodesAction(),
                
                // view control
                new PanCanvasAction(),
                new ZoomCanvasAction(),
                new ResetCanvasViewAction(),

                // global
                new BuildNodeTreeAction(),

                // behaviour
                new ActiveNodeBehaviourAction()

            };

            foreach (INodeEditorAction a in CanvasActions)
            {
                a.NodeEditor = this;
            }
        }

        public void CreateGUI()
        {
            BEventSystem = new BEventSystem<BNodeTreeEventBase>();

            SetupCanvasActions();

            this.SetAntiAliasing(8);

            EditorApplication.update -= Update;
            EditorApplication.update += Update;

            EditorApplication.playModeStateChanged -= HandlePlayModeStateChanged;
            EditorApplication.playModeStateChanged += HandlePlayModeStateChanged;

            Selection.selectionChanged -= HandleObjectSelection;
            Selection.selectionChanged += HandleObjectSelection;


            AllNodes = new List<NodeBaseElement>();
            AllLinks = new List<LinkElement>();
            SelectedNodes = new HashSet<NodeBaseElement>();
            CurrentNodeIDs = new List<int>();
            ZoomMinMax = new Vector2(0.1f, 10);

            // Import UXML
            VisualTreeAsset visualTree = AssetDatabase.LoadAssetAtPath<VisualTreeAsset>(UXML_PATH);
            VisualElement uxml = visualTree.Instantiate();

            StyleSheet customUss = AssetDatabase.LoadAssetAtPath<StyleSheet>(USS_PATH);

            // makes the container stretech to the rect of the window
            uxml.AddToClassList("w-100");
            uxml.AddToClassList("h-100");

            Root.Add(uxml);
            Root.styleSheets.Add(EditorConsts.GlobalStyleSheet);
            Root.styleSheets.Add(customUss);

            Canvas.transform.position = Vector2.zero;

            Intialize();

            foreach (INodeEditorAction a in CanvasActions)
            {
                a.OnEnable();
            }

            // Handle load data on startup
            EditorCoroutineUtility.StartCoroutine(StartupLoadData(), this);

            IsInitialized = true;
        }

        private IEnumerator StartupLoadData()
        {

            yield return CrtWaitCanvasLoad();

            // if we opened by clicking on an asset
            if (RequestOpen != null)
            {
                EditorCoroutineUtility.StartCoroutine(CrtSetSelectedTreeDataForce(RequestOpen), this);
                yield break;
            }

            // try reloading the selected object
            IBNodeTreeBehaviour tree = TryGetSelectedTreeBehaviour();

            if (tree != null)
            {
                EditorCoroutineUtility.StartCoroutine(CrtSetSelectedTreeBehaviourForce(tree), this);
                yield break;
            }

            NodeTreeData data = TryGetSelectedTreeData();

            if (data != null)
            {
                EditorCoroutineUtility.StartCoroutine(CrtSetSelectedTreeDataForce(data), this);
                yield break;
            }

            // else init with a default node node
            InitWithDefaultNode();

            EditorCoroutineUtility.StartCoroutine(CrtCenterCanvasView(), this);
        }

        private IEnumerator CrtCenterCanvasView()
        {
            while (true)
            {
                foreach (NodeBaseElement n in AllNodes)
                {
                    if (float.IsNaN(n.VisualElement.contentRect.width))
                        yield return null;
                }

                break;
            }

            new ResetCanvasViewAction() { NodeEditor = this }.Execute();
        }

        private void InitWithDefaultNode()
        {
            Type defaultNodeType = null;

            for (int i = 0; i < TypeUtils.AllTypes.Count; i++)
            {
                Type curr = TypeUtils.AllTypes[i];

                if (!curr.IsClass)
                    continue;

                if (curr.IsAbstract)
                    continue;

                if (!TypeUtils.IsSubTypeOf(curr, NodeBaseType))
                    continue;

                if (curr.GetAttribute<DefaultNodeAttribute>(true) == null)
                    continue;

                defaultNodeType = curr;
                break;
            }

            Assert.IsNotNull(defaultNodeType);

            INodeType firstNode = (INodeType)Activator.CreateInstance(defaultNodeType);
            firstNode.SetupInitialPorts();

            AddNode(firstNode, Vector3.zero);
        }

        private void HandlePlayModeStateChanged(PlayModeStateChange state)
        {
            if (!HasOpenInstances<BNodeTreeEditor>())
                return;

            // try reloading the selected object on play mode changed
            if (state == PlayModeStateChange.EnteredEditMode || state == PlayModeStateChange.EnteredPlayMode)
            {
                IBNodeTreeBehaviour tree = TryGetSelectedTreeBehaviour();
                if (tree != null)
                {
                    SelectedTreeBehaviourForce = tree;
                    return;
                }

                NodeTreeData data = TryGetSelectedTreeData();

                if (data != null)
                {
                    SelectedTreeDataForce = data;
                }
            }
        }




        private IBNodeTreeBehaviour TryGetSelectedTreeBehaviour()
        {
            IBNodeTreeBehaviour tree = null;

            for (int i = 0; i < Selection.objects.Length; i++)
            {
                UnityEngine.Object curr = Selection.objects[i];

                if (curr is GameObject g)
                {
                    tree = g.GetComponent<IBNodeTreeBehaviour>();
                    break;
                }
            }

            return tree;
        }


        private NodeTreeData TryGetSelectedTreeData()
        {
            NodeTreeData data = null;

            for (int i = 0; i < Selection.objects.Length; i++)
            {
                UnityEngine.Object curr = Selection.objects[i];

                if (curr is NodeTreeData d)
                {
                    data = d;
                    break;
                }
            }

            return data;
        }

        private void Intialize()
        {
            List<Type> validNodeTypes = TypeUtils.AllTypes
                .Where(t => t.IsAbstract)

                .Where(t => TypeUtils.IsSubTypeOf(t, typeof(INodeType)))
                .Where(t => t != typeof(INodeType))
                .Where(t => t != typeof(NodeBase<>))

                //.Where(t => t.BaseType == typeof(NodeBase<>).MakeGenericType(t))
                .ToList();

            if (validNodeTypes.Count == 0)
            {
                Debug.LogError($"You need to have atleast one class that inherits from {TypeUtils.GetNiceName(typeof(NodeBase<>))} in order to be able to use the node editor");
                Close();
                return;
            }

            NodeTypePicker = new PopupField<Type>("Node base type", validNodeTypes, 0, TypeUtils.GetNiceName, TypeUtils.GetNiceName);
            NodeTypePickerContainer.Add(NodeTypePicker);
            NodeBaseType = validNodeTypes[0];

            // data pickers
            NodeBehaviourPicker.objectType = typeof(BNodeTreeBehaviourBase);
            NodeTreeDataPicker.objectType = typeof(NodeTreeData);


            /*
            NodeBehaviourPicker.SetEnabled(false);
            NodeTreeDataPicker.SetEnabled(false);
            */

            // node type
            NodeTypePicker.RegisterValueChangedCallback(HandleNodeTypeChanged);

            SaveBtn.clickable.clicked += HandleSaveClicked;
            LoadBtn.clickable.clicked += HandleLoadClicked;
            Root.RegisterCallback<KeyDownEvent>(OnKeyPress);

            //// Register container events
            // scroll
            Container.RegisterCallback<ClickEvent>(OnMouseClick);
            Container.RegisterCallback<ContextClickEvent>(OnRightClick);
            Container.RegisterCallback<WheelEvent>(OnScrollWheel);
            Container.RegisterCallback<MouseDownEvent>(OnMouseDown);
            Container.RegisterCallback<MouseLeaveEvent>(OnMouseLeave);
            Container.RegisterCallback<MouseUpEvent>(OnMouseUp);
            Container.RegisterCallback<MouseMoveEvent>(OnMouseMove);
        }

        private void HandleNodeTypeChanged(ChangeEvent<Type> evt)
        {
            ClearCanvas();

            NodeBaseType = evt.newValue;
            NodeTypePicker.SetValueWithoutNotify(evt.newValue);

            InitWithDefaultNode();
        }


        public void AddNode(INodeType n, Vector2 nodePosition, Vector2? size = null, bool worldSpace = true)
        {
            
            // if no ID is assigned
            // then create one
            if (n.NodeID == -1)
            {
                int generatedID = -1;

                do
                {
                    generatedID = UnityEngine.Random.Range(0, MAX_NODE_ID);
                }
                while (CurrentNodeIDs.Contains(generatedID));

                n.NodeID = generatedID;
            }

            foreach (IPortType p in n.Ports)
            {
                p.LinkAttached = new List<ILinkType>();
            }

            NodeBaseElement node = new NodeBaseElement(n, this);

            // add nodes to list
            AllNodes.Add(node);
            
            CurrentNodeIDs.Add(n.NodeID);

            // add to ui

            node.BeforeAddToCanvas();

            Canvas.Add(node.VisualElement);
            node.VisualElement.transform.position = worldSpace ? WorldToCanvas(nodePosition) : (Vector2)nodePosition;

            node.AfterAddToCanvas();

            if (size.HasValue)
            {
                node.NodeSize = size.Value;
            }
        }



        public LinkElement AddLink(PortBaseElement from, PortBaseElement to)
        {
            // link type
            Type linkType = typeof(LinkDefault<>).MakeGenericType(NodeBaseType);

            ILinkType typedLink = (ILinkType)Activator.CreateInstance(linkType);
            typedLink.From = from.PortType;
            typedLink.To = to.PortType;

            to.PortType.LinkAttached.Add(typedLink);
            from.PortType.LinkAttached.Add(typedLink);

            // ui
            LinkElement link = new LinkElement(this, typedLink, from, to);

            Canvas.Add(link.VisualElement);
            AllLinks.Add(link);

            // links should always be behind the nodes
            link.VisualElement.SendToBack();

            link.AfterAddToCanvas();

            return link;
        }

        public void RemoveLink(ILinkType link)
        {
            LinkElement linkElem = AllLinks.FirstOrDefault(l => l.LinkType == link);
            AllLinks.Remove(linkElem);

            link.From.LinkAttached.Remove(link);
            link.To.LinkAttached.Remove(link);

            linkElem.BeforeRemoveFromCanvas();
            Canvas.Remove(linkElem.VisualElement);
        }

        public void RemoveNode(INodeType nodeType)
        {
            NodeBaseElement node = AllNodes.FirstOrDefault(n => n.NodeType == nodeType);
            // instance remove
            AllNodes.Remove(node);
            SelectedNodes.Remove(node);
            CurrentNodeIDs.Remove(node.NodeType.NodeID);

            node.BeforeRemoveFromCanvas();
            Canvas.Remove(node.VisualElement);

            for (int i = AllLinks.Count - 1; i >= 0; i--)
            {
                LinkElement curr = AllLinks[i];

                if (curr.From.ParentNode != node && curr.To.ParentNode != node)
                    continue;

                RemoveLink(curr.LinkType);
            }

            node.AfterRemoveFromCanvs();
        }

        #region save & load


        [OnOpenAsset]
        public static bool Open(EntityId instanceID, int line)
        {
            UnityEngine.Object obj = EditorUtility.EntityIdToObject(instanceID);

            if (!(obj is NodeTreeData data))
                return false;

            if (obj == null)
                return false;

            if (HasOpenInstances<BNodeTreeEditor>())
            {
                GetWindow<BNodeTreeEditor>().SelectedTreeData = data;
            }
            else
            {
                RequestOpen = data;
                GetWindow<BNodeTreeEditor>();
            }
            return true;
        }
        public bool Save()
        {
            var dialogPath = "Assets";
            var dialogFilename = "TreeNodeData";

            if(selectedNodeData != null)
            {
                string fullPath = AssetDatabase.GetAssetPath(selectedNodeData);
                dialogFilename = Path.GetFileName(fullPath);
                dialogPath = Path.GetDirectoryName(fullPath);
            }

            string savePath = EditorUtility.SaveFilePanel("Save tree data", dialogPath, dialogFilename, "asset");

            if (string.IsNullOrEmpty(savePath))
                return false;

            NodeTreeData treeData = GetTreeData();

            string projectPath = Application.dataPath;

            if (!savePath.StartsWith(projectPath))
                return false;

            savePath = FileUtil.GetProjectRelativePath(savePath);

            Debug.Log($"Tree Node Data Saved at : {savePath}");

            NodeTreeData existingAsset = AssetDatabase.LoadAssetAtPath<NodeTreeData>(savePath);

            if (existingAsset == null)
            {
                AssetDatabase.CreateAsset(treeData, savePath);
            }
            else
            {
                treeData.name = existingAsset.name;
                EditorUtility.CopySerializedIfDifferent(treeData, existingAsset);
                AssetDatabase.SaveAssets();
            }

            return true;
        }

        public NodeTreeData GetTreeData()
        {
            NodeTreeData treeData = CreateInstance<NodeTreeData>();
            treeData.NodeBaseType = NodeBaseType;
            List<NodeData> nodeData = treeData.Nodes;
            List<LinkData> linkData = treeData.Links;

            // add the nodes
            for (int i = 0; i < AllNodes.Count; i++)
            {
                NodeBaseElement curr = AllNodes[i];

                NodeData n = new NodeData()
                {
                    NodeType = curr.NodeType,
                    Position = curr.VisualElement.localBound.position,
                    Size = curr.NodeSize
                };

                nodeData.Add(n);
            }

            // add the links
            for (int i = 0; i < AllLinks.Count; i++)
            {
                LinkElement curr = AllLinks[i];
                INodeType nodeFrom = curr.From.ParentNode.NodeType;
                INodeType nodeTo = curr.To.ParentNode.NodeType;

                IPortType portFrom = curr.From.PortType;
                IPortType portTo = curr.To.PortType;

                LinkData l = new LinkData()
                {
                    //// nodes
                    // from
                    FromNodeIndex = nodeFrom.NodeID,
                    // to
                    ToNodeIndex = nodeTo.NodeID,

                    ///// ports
                    // from
                    FromPortIndex = nodeFrom.Ports.IndexOf(portFrom),
                    FromPortDirection = portFrom.PortDirection,
                    FromPortType = portFrom.PortType,
                    // to
                    ToPortIndex = nodeTo.Ports.IndexOf(portTo),
                    ToPortDirection = portTo.PortDirection,
                    ToPortType = portTo.PortType
                };

                linkData.Add(l);
            }

            return treeData;
        }

        public void Load(NodeTreeData data)
        {
            EditorCoroutineUtility.StartCoroutine(CrtLoad(data), this);
        }

        private IEnumerator CrtWaitCanvasLoad()
        {

            // wait for canvas construction
            while (Canvas == null || float.IsNaN(Canvas.contentRect.width))
            {
                yield return null;
            }

            yield break;
        }

        private IEnumerator CrtLoad(NodeTreeData data)
        {
            ClearCanvas();

            NodeTypePicker.SetValueWithoutNotify(data.NodeBaseType);
            NodeBaseType = data.NodeBaseType;


            yield return CrtWaitCanvasLoad();

            // create nodes
            foreach (NodeData n in data.Nodes)
            {
                foreach (IPortType o in n.NodeType.Ports)
                {
                    o.ParentNode = n.NodeType;
                    o.LinkAttached = new List<ILinkType>();
                }

                AddNode(n.NodeType, n.Position, n.Size, false);
            }

            // create link
            foreach (LinkData l in data.Links)
            {
                NodeBaseElement fromNode = AllNodes.FirstOrDefault(n => n.NodeType.NodeID == l.FromNodeIndex);
                NodeBaseElement toNode = AllNodes.FirstOrDefault(n => n.NodeType.NodeID == l.ToNodeIndex);

                PortBaseElement fromPort = fromNode.Ports[l.FromPortIndex];
                PortBaseElement toPort = toNode.Ports[l.ToPortIndex];

                AddLink(fromPort, toPort);
            }



            // wait for node reconstruction
            while (true)
            {
                for (int i = 0; i < AllNodes.Count; i++)
                {
                    NodeBaseElement n = AllNodes[i];
                    if (float.IsNaN(n.VisualElement.contentRect.width))
                        yield return null;
                }

                break;
            }

            // refresh links
            foreach (LinkElement l in AllLinks)
            {
                l.Refresh();
            }

            new ResetCanvasViewAction() { NodeEditor = this }.Execute();
        }

        #endregion


        #region utils

        public Vector2 WorldToCanvas(Vector2 pos)
        {
            return Canvas.WorldToLocal(pos);
        }

        public Vector2 WorldToContainer(Vector2 pos)
        {
            return Container.WorldToLocal(pos);
        }

        /// <summary>
        /// Is mouse inside the rect of a visual element ?
        /// </summary>
        /// <param name="pos">The world space position of the mouse (evt.position)</param>
        /// <returns></returns>
        public bool IsInsideNode(Vector2 pos)
        {
            foreach (NodeBaseElement n in AllNodes)
            {
                VisualElement curr = n.VisualElement;

                if (curr.worldBound.Contains(pos))
                    return true;
            }

            return false;
        }

        #endregion

        #region mouse & window events

        private void HandleLoadClicked()
        {
            string loadAssetPath = EditorUtility.OpenFilePanel("Open tree data", "Assets", "asset");

            string projectPath = Application.dataPath;

            if (!loadAssetPath.StartsWith(projectPath))
                return;

            loadAssetPath = "Assets" + loadAssetPath.Substring(projectPath.Length);


            NodeTreeData loadAsset = AssetDatabase.LoadAssetAtPath<NodeTreeData>(loadAssetPath);

            if (loadAsset == null)
                return;

            SelectedTreeDataForce = loadAsset;
        }

        private void HandleSaveClicked()
        {
            Save();
        }

        private void OnKeyPress(KeyDownEvent evt)
        {
            BEventSystem.Trigger(new OnWindowKeyPressed(this, evt));
        }

        private void OnRightClick(ContextClickEvent evt)
        {
            if (IsInsideNode(evt.mousePosition))
                return;

            BEventSystem.Trigger(new OnCanvasMouseContextClick(this, evt));
        }

        private void OnMouseClick(ClickEvent evt)
        {
            if (IsInsideNode(evt.position))
                return;

            BEventSystem.Trigger(new OnCanvasMouseClick(this, evt));
        }

        private void OnMouseMove(MouseMoveEvent evt)
        {
            BEventSystem.Trigger(new OnCanvasMouseMove(this, evt));
        }

        private void OnMouseUp(MouseUpEvent evt)
        {
            BEventSystem.Trigger(new OnCanvasMouseUp(this, evt));
        }

        private void OnMouseDown(MouseDownEvent evt)
        {
            BEventSystem.Trigger(new OnCanvasMouseDown(this, evt));
        }

        private void OnMouseLeave(MouseLeaveEvent evt)
        {
            BEventSystem.Trigger(new OnCanvasMouseLeave(this, evt));
        }

        private void OnScrollWheel(WheelEvent evt)
        {
            BEventSystem.Trigger(new OnCanvasScrollWheel(this, evt));
        }
        #endregion
    }
}