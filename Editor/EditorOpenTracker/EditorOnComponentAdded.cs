using Bloodthirst.Editor;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace Bloodthirst.Utils.EditorOpenTracker
{
    [InitializeOnLoad]
    public class EditorOnComponentAdded
    {
        private static IOnComponentAdded[] componentAddedHandlers = new IOnComponentAdded[]
        {
            new DisableRaycastTargetAndMaskOnComponentAdded(),
        };
        
        private static IOnGameObjectAdded[] gameObjectAddedHandlers = new IOnGameObjectAdded[]
        {
            new DisableRaycasTargetAndMaskOnGameObjectAdded(),
            new SetupButtonOnGameObjectAdded()
        };

        static EditorOnComponentAdded()
        {
            if (!EditorConsts.ON_ASSEMBLY_RELOAD_ON_COMPONENT_ADDED)
                return;

            ObjectFactory.componentWasAdded -= HandleComponentAdded;
            ObjectFactory.componentWasAdded += HandleComponentAdded;

            ObjectChangeEvents.changesPublished -= HandleObjectChange;
            ObjectChangeEvents.changesPublished += HandleObjectChange;

            EditorApplication.quitting -= OnEditorQuiting;
            EditorApplication.quitting += OnEditorQuiting;
        }

        private static void HandleObjectChange(ref ObjectChangeEventStream stream)
        {
            for(int i = 0; i < stream.length; i++)
            {
                ObjectChangeKind evtType = stream.GetEventType(i);

                if(evtType == ObjectChangeKind.ChangeGameObjectParent)
                {
                    stream.GetChangeGameObjectParentEvent(i, out ChangeGameObjectParentEventArgs data);
                    GameObject go = (GameObject)EditorUtility.EntityIdToObject(data.entityId);

                    for (int j = 0; j < gameObjectAddedHandlers.Length; j++)
                    {
                        gameObjectAddedHandlers[j].HandleGameObjectAdded(go);
                    }
                }

                if(evtType == ObjectChangeKind.ChangeGameObjectStructureHierarchy)
                {
                    stream.GetChangeGameObjectStructureHierarchyEvent(i, out ChangeGameObjectStructureHierarchyEventArgs data);
                    GameObject go = (GameObject)EditorUtility.EntityIdToObject(data.entityId);
                }
                if(evtType == ObjectChangeKind.CreateGameObjectHierarchy)
                {
                    stream.GetCreateGameObjectHierarchyEvent(i, out CreateGameObjectHierarchyEventArgs data);
                    GameObject go = (GameObject) EditorUtility.EntityIdToObject(data.entityId);

                }
            }
        }

        private static void HandleComponentAdded(UnityEngine.Component obj)
        {
            for (int i = 0; i < componentAddedHandlers.Length; i++)
            {
                componentAddedHandlers[i].HandleComponentAdded(obj);
            }
        }

        private static void OnEditorQuiting()
        {
            ObjectChangeEvents.changesPublished -= HandleObjectChange;
            ObjectFactory.componentWasAdded -= HandleComponentAdded;
            EditorApplication.quitting -= OnEditorQuiting;
        }
    }
}
