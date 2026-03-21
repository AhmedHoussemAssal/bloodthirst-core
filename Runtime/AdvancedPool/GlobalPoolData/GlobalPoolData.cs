using Bloodthirst.Core.PersistantAsset;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Pool;
using UnityEngine.Assertions;


#if UNITY_EDITOR
using UnityEditor;
#endif

namespace Bloodthirst.Core.AdvancedPool
{
    public struct GlobalPoolEntry
    {
        public GameObject Prefab;
        public int InitialSpawnCount;
    }

    public class GlobalPoolData : SingletonScriptableObject<GlobalPoolData>
    {
        [SerializeField]
        private GlobalPoolEntry[] poolEntries;
        public IReadOnlyList<GlobalPoolEntry> PoolEntries => poolEntries;

        public void Add(GameObject prefab, int count)
        {
            using (ListPool<GlobalPoolEntry>.Get(out List<GlobalPoolEntry> tmp))
            {
                tmp.AddRange(poolEntries);

                bool found = false;
                for (int i = tmp.Count - 1; i >= 0; i--)
                {
                    GlobalPoolEntry e = tmp[i];

                    if (e.Prefab == prefab)
                    {
                        found = true;

                        if (e.InitialSpawnCount != count)
                        {
                            int old = e.InitialSpawnCount;
                            e.InitialSpawnCount = count;
                            Debug.Log($"Prefab {prefab.name} already found , changed initiali spawn count from {old} to {count}");

                            tmp[i] = e;
                        }

                        break;
                    }
                }

                if (!found)
                {
                    tmp.Add(new GlobalPoolEntry() { Prefab = prefab, InitialSpawnCount = count });
                }

                poolEntries = tmp.ToArray();
            }
        }

        public void Remove(GameObject prefab)
        {
            using (ListPool<GlobalPoolEntry>.Get(out List<GlobalPoolEntry> tmp))
            {
                tmp.AddRange(poolEntries);

                for (int i = tmp.Count - 1; i >= 0; i--)
                {
                    GlobalPoolEntry e = tmp[i];

                    if (e.Prefab == prefab)
                    {
                        tmp.RemoveAt(i);
                    }
                }

                poolEntries = tmp.ToArray();
            }
        }

#if UNITY_EDITOR

        [MenuItem("Assets/Pooling/Add To Pool", true)]
        public static bool AddPrefabToPoolValidation()
        {
            if (Selection.activeObject == null) { return false; }

            if (!(Selection.activeObject is GameObject go)) { return false; }

            if (!PrefabUtility.IsPartOfPrefabAsset(go)) { return false; }


            GlobalPoolData pool = AssetDatabase.FindAssets($"t:{nameof(GlobalPoolData)}")
                .Select(g => AssetDatabase.LoadAssetByGUID<GlobalPoolData>(new GUID(g)))
                .FirstOrDefault();

            // Check if prefab already added
            foreach(GlobalPoolEntry e in pool.poolEntries)
            {
                if(e.Prefab == go) { return false; }
            }

            return true;
        }
        [MenuItem("Assets/Pooling/Add To Pool")]
        public static void AddPrefabToPool()
        {
            GameObject go = (GameObject)Selection.activeObject;

            GlobalPoolData pool = AssetDatabase.FindAssets($"t:{nameof(GlobalPoolData)}")
                .Select(g => AssetDatabase.LoadAssetByGUID<GlobalPoolData>(new GUID(g)))
                .FirstOrDefault();

            pool.Add(go, 10);

            EditorUtility.SetDirty(pool);
            AssetDatabase.SaveAssetIfDirty(pool);
            AssetDatabase.Refresh();

            EditorUtility.FocusProjectWindow();
            Selection.activeObject = pool;
            EditorGUIUtility.PingObject(pool);
        }
#endif
    }
}
