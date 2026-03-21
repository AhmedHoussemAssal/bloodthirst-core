using Bloodthirst.Editor;
using Bloodthirst.Editor.AssetProcessing;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using Microsoft.CSharp;
#if ODIN_INSPECTOR
	using Sirenix.Serialization;
#endif
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using UnityEditor;
using UnityEditor.Callbacks;
using UnityEngine;
using UnityEngine.Pool;
using UnityEngine.UIElements;

namespace Bloodthirst.Core.Utils
{

    [InitializeOnLoad]
    public static class AdvancedTypeCache
    {
        public class TypeInformation
        {
            
#if ODIN_INSPECTOR
[OdinSerialize]
#endif

            public Type type;

            
#if ODIN_INSPECTOR
[OdinSerialize]
#endif

            public TextAsset unityScript;

            
#if ODIN_INSPECTOR
[OdinSerialize]
#endif

            public string unityScriptPath;
        }

        private struct TextAssetPathPair
        {
            public MonoScript TextAsset { get; set; }
            public string AssetPath { get; set; }
            public string TextContent { get; set; }
        }

        private const string ASSET_PATH = "Packages/com.bloodthirst.bloodthirst-core/Editor/AdvancedTypeCache/AdvancedTypeCacheAsset.asset";

        private static AdvancedTypeCacheAsset cacheAsset;

        public static AdvancedTypeCacheAsset CacheAsset
        {
            get
            {
                if (cacheAsset == null)
                {
                    cacheAsset = new AdvancedTypeCacheAsset();
                    AssetDatabase.CreateAsset(cacheAsset, ASSET_PATH);
                }

                return cacheAsset;
            }
        }

        private static string pathToProject;

        private static CSharpCodeProvider provider;


        static AdvancedTypeCache()
        {
            if (!EditorConsts.ON_ASSEMBLY_RELOAD_ADVANCED_TYPE_CACHE)
                return;

            cacheAsset = AssetDatabase.LoadAssetAtPath<AdvancedTypeCacheAsset>(ASSET_PATH);
            pathToProject = EditorUtils.PathToProject;

            AssemblyReloadEvents.beforeAssemblyReload += BeforeReload;
            AssemblyReloadEvents.afterAssemblyReload += AfterReload;

            AssetWatcher.OnAssetCreated += HandleCreated;
            AssetWatcher.OnAssetMoved += HandleCreated;
            AssetWatcher.OnAssetEdited += HandleEdited;
            AssetWatcher.OnAssetRemoved += HandleRemoved;
        }

        private static void BeforeReload()
        {
            if (provider == null)
                return;

            provider.Dispose();
            provider = null;
        }

        private static void AfterReload()
        {
            provider = new CSharpCodeProvider();

            TreatRemovedScripts();
            TreatNewScripts();
        }

        private static void TreatNewScripts()
        {
            List<Type> allTypes = TypeUtils.AllTypes.ToList();

            for (int i = CacheAsset.newlyAddedScripts.Count - 1; i >= 0; i--)
            {
                string curr = CacheAsset.newlyAddedScripts[i];

                TextAsset scriptAsset = AssetDatabase.LoadAssetAtPath<TextAsset>(curr);

                if(scriptAsset == null)
                {
                    CacheAsset.newlyAddedScripts.RemoveAt(i);
                    continue;
                }

                List<BaseTypeDeclarationSyntax> types = TypesDeclaredInFile(scriptAsset.text);


                for (int j = 0; j < types.Count; j++)
                {
                    BaseTypeDeclarationSyntax c = types[j];
                    string classAsString = c.Identifier.Text;

                    Type findType = allTypes.FirstOrDefault(t => t.Name == classAsString);

                    // if type already exists
                    if (findType == null)
                    {
                        //Debug.Log($"New type to add now found {classAsString}");
                        continue;
                    }

                    if (cacheAsset.cache.TryGetValue(findType, out TypeInformation info))
                    {
                        //Debug.Log($"{findType.Name} already exists");
                    }
                    else
                    {
                        info = new TypeInformation();
                        cacheAsset.cache.Add(findType, info);
                    }

                    info.type = findType;
                    info.unityScriptPath = curr;
                    info.unityScript = scriptAsset;

                }

                // rempve from the newly added scripts
                CacheAsset.newlyAddedScripts.RemoveAt(i);
            }
        }

        private static void TreatRemovedScripts()
        {
            List<Type> typesToRemove = ListPool<Type>.Get();

            for (int i = CacheAsset.removedScripts.Count - 1; i >= 0; i--)
            {
                string currPath = CacheAsset.removedScripts[i];

                foreach (KeyValuePair<Type, TypeInformation> kv in CacheAsset.cache)
                {
                    if (kv.Value.unityScriptPath == currPath)
                    {
                        typesToRemove.Add(kv.Key);
                    }
                }

                // rempve from the newly added scripts
                CacheAsset.removedScripts.RemoveAt(i);
            }

            foreach (Type k in typesToRemove)
            {
                CacheAsset.cache.Remove(k);
            }

            ListPool<Type>.Release(typesToRemove);
        }

        private static void HandleRemoved(AssetImporter importer)
        {
            if (!(importer is MonoImporter monoImporter))
                return;

            cacheAsset.removedScripts.Add(monoImporter.assetPath);
        }

        private static void HandleCreated(AssetImporter importer)
        {
            if (!(importer is MonoImporter monoImporter))
                return;

            cacheAsset.newlyAddedScripts.Add(monoImporter.assetPath);
        }

        private static void HandleEdited(AssetImporter importer)
        {
            if (!(importer is MonoImporter monoImporter))
                return;

            cacheAsset.newlyAddedScripts.Add(monoImporter.assetPath);
            cacheAsset.removedScripts.Add(monoImporter.assetPath);
        }

        internal static void StartThread()
        {
            List<TextAssetPathPair> deps = new List<TextAssetPathPair>(FetchUnityAssetsInFolders());

            Thread t = new(() =>
            {
                cacheAsset.cache = ExtractInfo(deps);
                cacheAsset.newlyAddedScripts.Clear();
            });

            t.Start();
        }

        private static IEnumerable<TextAssetPathPair> FetchUnityAssetsInFolders()
        {
            List<MonoScript> assets = new List<MonoScript>();
            string[] paths = cacheAsset.GetAssemblyPaths().ToArray();
            string[] guids = AssetDatabase.FindAssets("t:MonoScript" , paths);

            for (int i = 0; i < guids.Length; i++)
            {
                MonoScript asset = AssetDatabase.LoadAssetByGUID<MonoScript>(new GUID(guids[i]));

                if (asset == null)
                    continue;

                yield return new TextAssetPathPair() { TextAsset = asset, AssetPath = AssetDatabase.GetAssetPath(asset), TextContent = asset.text };
            }

        }


        private static List<BaseTypeDeclarationSyntax> TypesDeclaredInFile(string fileContent)
        {
            List<BaseTypeDeclarationSyntax> typesList = new List<BaseTypeDeclarationSyntax>();

            SyntaxTree syntaxTree = CSharpSyntaxTree.ParseText(fileContent);

            CompilationUnitSyntax root = syntaxTree.GetRoot() as CompilationUnitSyntax;

            // get classes inside namespaces
            foreach (NamespaceDeclarationSyntax s in root.Members.OfType<NamespaceDeclarationSyntax>())
            {
                typesList.AddRange(s.Members.OfType<ClassDeclarationSyntax>());
                typesList.AddRange(s.Members.OfType<StructDeclarationSyntax>());
                typesList.AddRange(s.Members.OfType<EnumDeclarationSyntax>());
                typesList.AddRange(s.Members.OfType<InterfaceDeclarationSyntax>());
            }

            // get classes in file directly
            foreach (BaseTypeDeclarationSyntax s in root.Members.OfType<BaseTypeDeclarationSyntax>())
            {
                typesList.Add(s);
            }

            return typesList;
        }

        /// <summary>
        /// Extracts info about the classes that follow the BISD pattern
        /// </summary>
        /// <param name="TypeList"></param>
        /// <param name="TextList"></param>
        private static Dictionary<Type, TypeInformation> ExtractInfo(List<TextAssetPathPair> deps)
        {
            CSharpCodeProvider provider = new CSharpCodeProvider();
            int id = Progress.Start("Registering AdvancedTypeCahce", null, Progress.Options.None);

            Progress.Report(id, 0f);

            try
            {
                Dictionary<Type, TypeInformation> typeList = new Dictionary<Type, TypeInformation>();

                List<Type> allTypes = TypeUtils.AllTypes.ToList();

                for (int i = 0; i < deps.Count; i++)
                {
                    TextAssetPathPair d = deps[i];
                    string relativePath = d.AssetPath;

                    string systemPath = pathToProject + relativePath;

                    List<BaseTypeDeclarationSyntax> typesList = TypesDeclaredInFile(d.TextContent);

                    foreach (BaseTypeDeclarationSyntax c in typesList)
                    {
                        string classAsString = c.Identifier.Text;

                        Type findType = allTypes.FirstOrDefault(t => t.Name == classAsString);

                        if (findType != null)
                        {
                            typeList.TryAdd(findType, new TypeInformation()
                            {
                                type = findType,
                                unityScriptPath = relativePath,
                                unityScript = d.TextAsset
                            });
                        }
                    }

                    Progress.Report(id, i / (float)(deps.Count - 1));

                }

                return typeList;
            }

            catch (Exception e)
            {
                Debug.LogException(e);
            }
            finally
            {
                provider.Dispose();
                Progress.Remove(id);
            }

            return null;

        }
    }
}
