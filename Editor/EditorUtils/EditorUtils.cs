#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using UnityEditor;
using UnityEditor.Experimental;
using UnityEditorInternal;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.Pool;
using UnityEngine.UIElements;

namespace Bloodthirst.Core.Utils
{
    /// <summary>
    /// an extension class containing helper methods for types / reflection
    /// </summary>
    public static class EditorUtils
    {
        private static Func<string> GetActiveFolderPathCached;

        /// <summary>
        /// Returns the folder selected in the project tab
        /// </summary>
        /// <returns></returns>
        public static string GetProjectTabPath()
        {
            if (GetActiveFolderPathCached == null)
            {
                GetActiveFolderPathCached = (Func<string>)typeof(ProjectWindowUtil)
                    .GetMethod("GetActiveFolderPath", BindingFlags.Static | BindingFlags.NonPublic)
                    .CreateDelegate(typeof(Func<string>));
            }

            return GetActiveFolderPathCached();
        }


        public static Texture GetIcon(UnityEngine.Object obj)
        {
            Texture tex = AssetPreview.GetMiniThumbnail(obj);
            return tex;
        }

        public static IEnumerable<T> FindAssetsByType<T>(params string[] paths) where T : UnityEngine.Object
        {
            return FindAssetsAs<T>($"t:{typeof(T).Name}", paths);
        }

        public static IEnumerable<T> FindAssetsAs<T>(string filter, params string[] paths) where T : UnityEngine.Object
        {
            string[] assetGuids = AssetDatabase.FindAssets(filter, paths);

            for (int i = 0; i < assetGuids.Length; i++)
            {
                string guid = assetGuids[i];
                T asset = AssetDatabase.LoadAssetByGUID<T>(new GUID(guid));

                yield return asset;
            }
        }

        [MenuItem("Bloodthirst Tools/Shortcuts/Reserialize selected")]
        public static void ReserializeSelected()
        {
            if(Selection.objects.Length == 0)
            {
                EditorUtility.DisplayDialog("Error", "Atleast one asset needs to be selected", "Yes");
                return;
            }

            using (ListPool<UnityEngine.Object>.Get(out List<UnityEngine.Object> selected))
            using (ListPool<string>.Get(out List<string> paths))
            {
                StringBuilder sb = new StringBuilder();

                selected.AddRange(Selection.objects);

                sb.AppendLine("Are you sure you want to re-serialize all these assets ?");
                foreach (UnityEngine.Object g in selected)
                {
                    string p = AssetDatabase.GetAssetPath(g);
                    paths.Add(p);
                    sb.AppendLine(p);
                }

                bool userAnswer = EditorUtility.DisplayDialog("Confirmation", sb.ToString(), "Yes", "No");

                if (!userAnswer)
                {
                    return;
                }

                AssetDatabase.ForceReserializeAssets(paths, ForceReserializeAssetsOptions.ReserializeAssetsAndMetadata);

                EditorUtility.DisplayDialog("Sucess", "Reserialization done", "Yes");
            }
        }

        /// <summary>
        /// Clears the console in the editor
        /// </summary>
        public static void ClearConsole()
        {
            Assembly assembly = Assembly.GetAssembly(typeof(SceneView));
            Type type = assembly.GetType("UnityEditor.LogEntries");
            MethodInfo method = type.GetMethod("Clear");
            method.Invoke(new object(), null);
        }

        public static bool IsLayoutBuilt(this VisualElement root)
        {
            return !root.Query<VisualElement>()
                 .Build()
                 .Any(v => float.IsNaN(v.layout.width) || float.IsNaN(v.layout.height));
        }

        public static void Display(this VisualElement visualElement, bool show)
        {
            DisplayStyle style = show ? DisplayStyle.Flex : DisplayStyle.None;
            visualElement.style.display = style;
        }

        public static string CurrentProjectWindowPath()
        {
            Type projectWindowUtilType = typeof(ProjectWindowUtil);
            MethodInfo getActiveFolderPath = projectWindowUtilType.GetMethod("GetActiveFolderPath", BindingFlags.Static | BindingFlags.NonPublic);
            object obj = getActiveFolderPath.Invoke(null, new object[0]);
            string pathToCurrentFolder = obj.ToString();

            return pathToCurrentFolder;
        }

        public static string GetFolderContainingAsset(UnityEngine.Object asset)
        {
            string originalPath = AssetDatabase.GetAssetPath(asset);
            string originalFolderPath = Path.GetDirectoryName(originalPath).Replace("\\", "/");

            return originalFolderPath;
        }

        public static string GetFolderFromPath(string path)
        {
            bool isFilePath = false;
            for (int i = path.Length - 1; i > -1; i--)
            {
                if (path[i] == '.')
                {
                    isFilePath = true;
                    continue;
                }

                if (path[i] == '/')
                {
                    if (isFilePath)
                        return path.Substring(0, i);
                    else
                        break;
                }
            }

            return path;
        }

        /// <summary>
        /// <para>Create a folder based on path</para>
        /// <para>Works if the path ends with filename + extension also</para>
        /// <para>example : "Assets/Resources/Foo/Bar"</para>
        /// <para>OR : "Assets/Resources/Foo/Bar/SomeAsset.asset"</para>
        /// </summary>
        /// <param name="path"></param>
        public static void CreateFoldersFromPath(string path)
        {
            List<string> folders = path.Split('/').ToList();

            if (folders.Last().Contains('.'))
            {
                folders.RemoveAt(folders.Count - 1);
            }

            // the path must start from assets
            Assert.AreEqual(folders[0], "Assets");

            string prevFolder = folders[0];

            for (int i = 1; i < folders.Count; i++)
            {
                string folderToCheck = prevFolder + "/" + folders[i];

                if (!AssetDatabase.IsValidFolder(folderToCheck))
                {
                    AssetDatabase.CreateFolder(prevFolder, folders[i]);
                }

                prevFolder = folderToCheck;
            }
        }

        private static string pathToProject;

        /// <summary>
        /// <para>Takes a relative paht and returns it as absolute path</para>
        /// <para>Relative paths are usually returned from <see cref="AssetDatabase.GetAssetPath(int)"/> or other <see cref="AssetDatabase"/> methods </para>
        /// </summary>
        /// <param name="relative">The relative path , usually the path you get from <see cref="AssetDatabase.GetAssetPath(int)"/> </param>
        /// <returns>The absolute path</returns>
        public static string RelativeToAbsolutePath(string relative)
        {
            return PathToProject + "/" + relative;
        }


        /// <summary>
        /// <para>Takes an anbsolute path and returns it as relative path</para>
        /// <para>Relative paths are usually used in methods provided in <see cref="AssetDatabase"/> </para>
        /// </summary>
        /// <param name="relative">The relative path , usually the path you get from <see cref="AssetDatabase.GetAssetPath(int)"/> </param>
        /// <returns>The relative path OR <see cref="null"/> if the path isn't in the project</returns>
        public static string AbsoluteToRelativePath(string absolute)
        {
            string relative = null;

            if (absolute.StartsWith(Application.dataPath))
            {
                relative = "Assets" + absolute.Substring(Application.dataPath.Length);
            }

            return relative;
        }

        /// <summary>
        ///<para>Path to project (without the Asset folder)</para>
        ///<para>Example : C:/UnityProjects/[ProjectName]</para>
        /// </summary>
        public static string PathToProject
        {
            get
            {
                if (string.IsNullOrEmpty(pathToProject))
                {
                    pathToProject = Application.dataPath;
                    pathToProject = pathToProject.Substring(0, pathToProject.Length - "/Assets".Length);
                }

                return pathToProject;
            }
        }

        /// <summary>
        /// Get all the text assets in the project , this includes scripts
        /// </summary>
        /// <returns></returns>
        public static List<MonoScript> FindScriptAssets()
        {
            List<MonoScript> assets = new List<MonoScript>();
            string[] guids = AssetDatabase.FindAssets("t:MonoScript");
            for (int i = 0; i < guids.Length; i++)
            {
                MonoScript asset = AssetDatabase.LoadAssetByGUID<MonoScript>(new GUID(guids[i]));

                if (asset != null)
                {
                    assets.Add(asset);
                }
            }
            return assets;
        }

        /// <summary>
        /// Get all scene in the project (open AND closed)
        /// </summary>
        /// <returns></returns>
        public static List<string> GetAllScenePathsInProject()
        {
            // load scene assets
            string[] scenesGUIDs = AssetDatabase.FindAssets("t:Scene");

            List<string> editorBuildSettingsScenes = new List<string>();

            foreach (string sceneGUID in scenesGUIDs)
            {
                string scenePath = AssetDatabase.GUIDToAssetPath(sceneGUID);

                // if scene is valid add it to the scenes list in build settings
                editorBuildSettingsScenes.Add(scenePath);
            }

            return editorBuildSettingsScenes;
        }


        /// <summary>
        /// Get a path to an asset placed in a resource folder
        /// </summary>
        /// <param name="poolablePrefab"></param>
        /// <returns></returns>
        public static string GetResourcesPath(UnityEngine.Object poolablePrefab)
        {
            var path = AssetDatabase.GetAssetPath(poolablePrefab);

            if (string.IsNullOrEmpty(path))
                return null;

            if (!path.StartsWith("Resources/"))
            {
                Debug.LogWarning($"The prefab {poolablePrefab.name} isn't placed in the Resources folder");
                return path;
            }

            int index = path.LastIndexOf("Resources/", path.Length - 1);

            path = path.Substring(index + "Resources/".Length);

            if (index == -1)
                return path;

            int extensionDot = path.LastIndexOf(".", path.Length - 1);

            if (extensionDot == -1)
                return path;

            return path.Substring(0, extensionDot);

        }
    }
}
#endif
