using Bloodthirst.BJson;
using Bloodthirst.Core.Utils;

#if ODIN_INSPECTOR
using Sirenix.OdinInspector;
#endif
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine.Assertions;
using System.Reflection;
using Sirenix.Serialization;
using System.Diagnostics;
using Sirenix.Serialization.Utilities;
using System.IO;
using UnityEngine.Pool;

namespace Bloodthirst.Runtime.BNodeTree
{
    public class NodeTreeData : SerializedScriptableObject
    {




#if ODIN_INSPECTOR                           [ShowInInspector]
        [OdinSerialize]
#endif
        private Type nodeBaseType;

#if ODIN_INSPECTOR
        [ShowInInspector]
        [OdinSerialize]
#endif
        [BJsonIgnore]
        private List<NodeData> nodes;




#if ODIN_INSPECTOR        [ShowInInspector]
        [OdinSerialize]
#endif
        [BJsonIgnore]
        private List<LinkData> links;

        public Type NodeBaseType { get => nodeBaseType; set => nodeBaseType = value; }
        public List<NodeData> Nodes { get => nodes; set => nodes = value; }
        public List<LinkData> Links { get => links; set => links = value; }

        public NodeTreeData()
        {
            nodes = new List<NodeData>();
            links = new List<LinkData>();
        }

#if UNITY_EDITOR
        private void OnValidate()
        {
            for (int i = 0; i < nodes.Count; i++)
            {
                NodeData node = nodes[i];
                Type t = node.NodeType.GetType();

                BindingFlags flags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;
                MethodInfo validateMethod = t.GetMethod("OnValidate", flags);

                if (validateMethod == null)
                    continue;

                validateMethod.Invoke(node.NodeType, Array.Empty<object>());

            }
        }
#endif

        private static T DeepCopy<T>(T src)
        {
            using MemoryStream mem = new MemoryStream(1024 * 16);
            using Cache<SerializationContext> cache = Cache<SerializationContext>.Claim();
            using Cache<DeserializationContext> cache2 = Cache<DeserializationContext>.Claim();
            cache.Value.Config.SerializationPolicy = SerializationPolicies.Everything;
            cache2.Value.Config.SerializationPolicy = SerializationPolicies.Everything;
            SerializationUtility.SerializeValue(src, mem, DataFormat.Binary, out var unityObjects, cache);
            mem.Position = 0L;
            return SerializationUtility.DeserializeValue<T>(mem, DataFormat.Binary, unityObjects, cache2);
        }

        public void BuildAllNodes<TNode>(List<TNode> allNodes) where TNode : INodeType
        {
            Assert.IsTrue(TypeUtils.IsSubTypeOf(NodeBaseType, typeof(TNode)), $"You're trying to get a node structure of type {NodeBaseType.Name} as use it as {typeof(TNode).Name}");

            if (allNodes.Capacity < Nodes.Count)
            {
                allNodes.Capacity = Nodes.Count;
            }

            // copy the nodes
            foreach (NodeData n in Nodes)
            {
                // doesn't work for IL2CPP
                // TNode nodeType = BCopier<TNode>.Instance.Copy((TNode)n.NodeType);
                try
                {
                    //TNode nodeType = (TNode)SerializationUtility.CreateCopy((TNode)n.NodeType);
                    TNode nodeType = DeepCopy((TNode)n.NodeType);

                    foreach (IPortType p in nodeType.Ports)
                    {
                        p.LinkAttached = new List<ILinkType>();
                    }

                    allNodes.Add(nodeType);

                }
                catch (Exception ex)
                {
                    UnityEngine.Debug.LogException(ex);
                }

            }

            // link the port references
            foreach (LinkData l in Links)
            {
                // get the 2 nodes linked
                TNode fromNode = allNodes.FirstOrDefault(n => n.NodeID == l.FromNodeIndex);
                TNode toNode = allNodes.FirstOrDefault(n => n.NodeID == l.ToNodeIndex);

                // get the ports of the nodes linked
                IPortType fromPort = ((INodeType)fromNode).Ports[l.FromPortIndex];
                IPortType toPort = ((INodeType)toNode).Ports[l.ToPortIndex];

                // assign nodes as parents of the ports
                fromPort.ParentNode = fromNode;
                toPort.ParentNode = toNode;

                // create the link
                LinkDefault<TNode> link = new LinkDefault<TNode>()
                {
                    From = fromPort,
                    To = toPort
                };

                // attach the link
                fromPort.LinkAttached.Add(link);
                toPort.LinkAttached.Add(link);
            }

        }

        /// <summary>
        /// Create a copy of the node tree structure
        /// </summary>
        /// <returns></returns>
        public IEnumerable<TNode> BuildAllNodes<TNode>() where TNode : INodeType
        {
            Assert.IsTrue(TypeUtils.IsSubTypeOf(NodeBaseType, typeof(TNode)), $"You're trying to get a node structure of type {NodeBaseType.Name} as use it as {typeof(TNode).Name}");

            // get the nodes
            using (ListPool<TNode>.Get(out var allNodes))
            {
                BuildAllNodes(allNodes);

                foreach (TNode n in allNodes)
                {
                    yield return n;
                }
            }
        }
    }
}




