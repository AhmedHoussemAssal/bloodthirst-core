using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.Events;
using UnityEngine.Pool;
using UnityEngine.SceneManagement;
using UnityEngine.UIElements;

namespace Bloodthirst.Core.Utils
{
    public static class GameObjectUtils
    {
        [Flags]
        public enum ResetFlag
        {
            PositionAndRotation,
            Scale,
            All
        }
        public static bool HasComponent<T>(this GameObject curr) where T : Component
        {
            return curr.TryGetComponent(out T _);
        }


        public static bool HasComponent<T>(this Component curr) where T : Component
        {
            return curr.TryGetComponent(out T _);
        }

        public static void ResetTransform(this Transform t, ResetFlag flag = ResetFlag.All)
        {
            switch (flag)
            {
                case ResetFlag.PositionAndRotation: { t.SetLocalPositionAndRotation(Vector3.zero, Quaternion.identity); return; }
                case ResetFlag.Scale: { t.localScale = Vector3.one; return; }
                case ResetFlag.All: { t.SetLocalPositionAndRotation(Vector3.zero, Quaternion.identity); t.localScale = Vector3.one; return; }
                default: { throw new NotImplementedException($"Missing case {flag}"); }
            }
        }

        public static bool HasDuplicateSubscriptions(UnityEventBase unityEvt)
        {
            bool hasDuplicate = false;

            using (HashSetPool<string>.Get(out var tmp))
            {
                int count = unityEvt.GetPersistentEventCount();
                for (int i = 0; i < count; i++)
                {
                    string evt = unityEvt.GetPersistentMethodName(i);

                    if (!tmp.Add(evt))
                    {
                        Debug.LogError($"Method named {evt} has been added more than once to {nameof(ClickEvent)}");
                        hasDuplicate = true;
                    }
                }
            }

            return hasDuplicate;
        }

        public static bool IsEmptyDelegate(Delegate evt)
        {
            return evt == null;
        }

        public static bool HasDuplicateSubscriptions(Delegate evt)
        {
            if (evt == null)
            {
                return false;
            }

            bool hasDuplicate = false;

            using (HashSetPool<Delegate>.Get(out HashSet<Delegate> tmp))
            {
                Delegate[] callbacks = evt.GetInvocationList();

                for (int i = 0; i < callbacks.Length; i++)
                {
                    Delegate callback = callbacks[i];

                    if (!tmp.Add(evt))
                    {
                        Debug.LogError($"Method named {callback.Method.Name} has been added more than once to {evt.Method} at {evt.Target}");
                        hasDuplicate = true;
                    }
                }
            }

            return hasDuplicate;
        }

        public static string GetReadableElapsedTime(DateTime from, DateTime to)
        {
            const int SECOND = 1;
            const int MINUTE = 60 * SECOND;
            const int HOUR = 60 * MINUTE;
            const int DAY = 24 * HOUR;
            const int MONTH = 30 * DAY;

            var ts = new TimeSpan(to.Ticks - from.Ticks);
            double delta = Math.Abs(ts.TotalSeconds);

            if (delta < 1 * MINUTE)
                return ts.Seconds == 1 ? "one second ago" : ts.Seconds + " seconds ago";

            if (delta < 2 * MINUTE)
                return "a minute ago";

            if (delta < 45 * MINUTE)
                return ts.Minutes + " minutes ago";

            if (delta < 90 * MINUTE)
                return "an hour ago";

            if (delta < 24 * HOUR)
                return ts.Hours + " hours ago";

            if (delta < 48 * HOUR)
                return "yesterday";

            if (delta < 30 * DAY)
                return ts.Days + " days ago";

            if (delta < 12 * MONTH)
            {
                int months = Convert.ToInt32(Math.Floor((double)ts.Days / 30));
                return months <= 1 ? "one month ago" : months + " months ago";
            }
            else
            {
                int years = Convert.ToInt32(Math.Floor((double)ts.Days / 365));
                return years <= 1 ? "one year ago" : years + " years ago";
            }
        }

        public static string GetGameObjectScenePath(GameObject gameObject)
        {
            StringBuilder sb = new StringBuilder();

            Transform curr = gameObject.transform;

            using (ListPool<string>.Get(out List<string> tmp))
            {
                while (curr != null)
                {
                    tmp.Add(curr.name);
                    curr = curr.parent;
                }

                tmp.Reverse();

                sb.Append(gameObject.scene.name);

                foreach (string s in tmp)
                {
                    sb.Append('/');
                    sb.Append(s);
                }
            }

            return sb.ToString();
        }

        public static void GetAllRootGameObjects(List<GameObject> lst)
        {
            using (ListPool<GameObject>.Get(out List<GameObject> cache))
            {
                for (int i = 0; i < UnityEngine.SceneManagement.SceneManager.sceneCount; i++)
                {
                    UnityEngine.SceneManagement.Scene scene = UnityEngine.SceneManagement.SceneManager.GetSceneAt(i);

                    scene.GetRootGameObjects(cache);

                    lst.AddRange(cache);
                }
            }
        }

        public static void MoveToScene(GameObject go, Scene scene)
        {
            SceneManager.MoveGameObjectToScene(go, scene);
        }

        public static void GetAllComponents<T>(ref List<T> list, bool includeInactive)
        {
            list = CollectionsUtils.CreateOrClear(list);

            using (ListPool<GameObject>.Get(out var gos))
            {
                GetAllRootGameObjects(gos);

                GetAllComponents(ref list, gos, includeInactive);
            }
        }

        public static void GetAllComponents<T>(ref List<T> list, IReadOnlyList<GameObject> go, bool includeInactive)
        {
            using (ListPool<T>.Get(out List<T> tmp))
            {
                for (int i = 0; i < go.Count; i++)
                {
                    GameObject rootGameObject = go[i];

                    tmp.Clear();
                    rootGameObject.GetComponentsInChildren(includeInactive, tmp);
                    list.AddRange(tmp);
                }
            }
        }

        /// <summary>
        /// Combines a list of coroutines into one with the same order in the params
        /// </summary>
        /// <param name="enumerators"></param>
        /// <returns></returns>
        public static IEnumerator CombineCoroutine(params IEnumerator[] enumerators)
        {
            foreach (IEnumerator c in enumerators)
            {
                while (c.MoveNext())
                {
                    yield return c.Current;
                }
            }
        }
        public static void ClearTransform(this Transform t)
        {
            for (int i = t.childCount - 1; i >= 0; i--)
            {
                UnityEngine.Object.Destroy(t.GetChild(i).gameObject);
            }
        }

        public static void ClearTransformEditor(this Transform t)
        {
            for (int i = t.childCount - 1; i >= 0; i--)
            {
                UnityEngine.Object.DestroyImmediate(t.GetChild(i).gameObject);
            }
        }

        public static void SetCollisionBetweenObjects(GameObject a, GameObject b, bool enableCollision)
        {
            Assert.IsTrue(a != b);
            
            using (ListPool<Collider>.Get(out List<Collider> aCols))
            using (ListPool<Collider>.Get(out List<Collider> bCols))
            {
                a.GetComponentsInChildren(aCols);
                b.GetComponentsInChildren(bCols);

                Assert.IsTrue(aCols.Intersect(bCols).Count() == 0);

                foreach (Collider bCol in bCols)
                {
                    foreach (Collider aCol in aCols)
                    {
                        Assert.IsTrue(aCol != bCol);
                        Physics.IgnoreCollision(aCol, bCol, !enableCollision);
                    }
                }
            }
        }
    }
}
