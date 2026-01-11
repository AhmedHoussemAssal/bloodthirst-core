using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine.Assertions;
using UnityEngine.Pool;

namespace Bloodthirst.Core.Utils
{
    /// <summary>
    /// an extension class containing helper methods for collections
    /// </summary>
    public static class CollectionsUtils
    {
        public static T GetRandom<T>(this IReadOnlyList<T> lst)
        {
            Assert.IsTrue(lst.Count != 0);

            int idx = UnityEngine.Random.Range(0, lst.Count);

            return lst[idx];
        }

        public static T GetRandom<T>(this IReadOnlyList<T> lst , out int idx)
        {
            Assert.IsTrue(lst.Count != 0);

            idx = UnityEngine.Random.Range(0, lst.Count);

            return lst[idx];
        }

        /// <summary>
        /// <para>Shift every element in the list by <paramref name="rotationOffset"/> , behaving like a ring-buffer or a circular-linked-list </para>
        /// <para>if an element goes out of bounds (beyond 0 or list.Count) , it loops backs to the start or end , similar to a carouselle </para>
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="list"></param>
        /// <param name="rotationOffset"></param>
        public static void Rotate<T>(this IList<T> list, int rotationOffset)
        {
            int count = list.Count;

            if (count == 0)
            {
                return;
            }

            rotationOffset = rotationOffset % count;

            if (rotationOffset == 0)
            {
                return;
            }

            using (ListPool<T>.Get(out List<T> tmp))
            {
                for (int i = 0; i < count; ++i)
                {
                    int curr = (i + count + rotationOffset) % count;

                    tmp.Add(list[curr]);
                }

                for (int i = 0; i < tmp.Count; i++)
                {
                    list[i] = tmp[i];
                }
            }
        }

        /// <summary>
        /// Comapres two collections and returns true if both of them contains the same elements (without considering the order of elements)
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static bool CollectionEquals<T>(IReadOnlyList<T> a , IReadOnlyList<T> b)
        {
            if(a.Count != b.Count)
            {
                return false;
            }

            for (int i = 0; i < a.Count; i++)
            {
                T el1 = a[i];
                bool found = false;

                for (int i1 = 0; i1 < b.Count; i1++)
                {
                    T el2 = b[i1];

                    if(el1.Equals(el2))
                    {
                        found = true;
                        break;
                    }
                }

                if (!found)
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Comapres two collections and returns true if both of them contains the same elements (without considering the order of elements)
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static bool CollectionEquals<T>(IEnumerable<T> a, IEnumerable<T> b)
        {
            using(ListPool<T>.Get(out List<T> aList))
            using(ListPool<T>.Get(out List<T> bList))
            {
                aList.AddRange(a);
                bList.AddRange(b);

                return CollectionEquals(aList, bList);
            }
        }


        public static void Shuffle<T>(this IList<T> list , Random rng )
        {
            int n = list.Count;
            while (n > 1)
            {
                n--;
                int k = rng.Next(n + 1);
                T value = list[k];
                list[k] = list[n];
                list[n] = value;
            }
        }

        /// <summary>
        /// Collection case to linq
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="collectionBase"></param>
        /// <returns></returns>
        public static IEnumerable<T> CollectionToLinq<T>(this CollectionBase collectionBase)
        {
            foreach (object i in collectionBase)
            {
                yield return (T)i;
            }
        }


        /// <summary>
        /// Used to combine coroutines
        /// </summary>
        /// <param name="curr"></param>
        /// <param name="other"></param>
        /// <returns></returns>
        public static IEnumerator Combine(this IEnumerator curr, IEnumerator other)
        {
            while (curr.MoveNext())
            {
                yield return curr.Current;
            }

            while (other.MoveNext())
            {
                yield return other.Current;
            }
        }

        public static bool IsSame<T>(this IList<T> a, IList<T> b) where T : IEquatable<T>
        {
            if (a == b)
                return true;

            if (a == null || b == null)
                return false;

            if (a.Count != b.Count)
                return false;

            for (int i = 0; i < a.Count; i++)
            {
                if (!a[i].Equals(b[i]))
                    return false;
            }

            return true;
        }
        public static string ReplaceInterval(this string value, int startIndex, int endIndex, char replacementChar)
        {
            char[] chatArray = value.ToCharArray();

            for (int i = startIndex; i < endIndex; i++)
            {
                chatArray[i] = replacementChar;
            }

            return new string(chatArray);
        }

        /// <summary>
        /// Index of for ReadOnlyList
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="list"></param>
        /// <param name="item"></param>
        /// <returns></returns>
        public static int IndexOf<T>(this IReadOnlyList<T> list, Predicate<T> filter)
        {
            for (int i = 0; i < list.Count; i++)
            {
                if (filter(list[i])) return i;
            }
            return -1;
        }

        /// <summary>
        /// Index of for ReadOnlyList
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="list"></param>
        /// <param name="item"></param>
        /// <returns></returns>
        public static int IndexOf<T>(this IReadOnlyList<T> list, T item)
        {
            for (int i = 0; i < list.Count; i++)
            {
                if (list[i].Equals(item)) return i;
            }
            return -1;
        }

        /// <summary>
        /// Add a new key-value pair to the dictionary if the key doesnt exist
        /// </summary>
        /// <typeparam name="K">TKey</typeparam>
        /// <typeparam name="V">TValue</typeparam>
        /// <param name="dict">dictionaty</param>
        /// <param name="key">Key value</param>
        public static void AddKeyValue<K, V>(this Dictionary<K, V> dict, K key) where V : new()
        {
            if (!dict.ContainsKey(key))
            {
                dict.Add(key, new V());
            }
        }

        public static V GetOrCreateValue<K, V>(this IDictionary<K, V> dict, K key) where V : new()
        {
            if (!dict.TryGetValue(key, out V val))
            {
                val = new V();
                dict.Add(key, val);
                return val;
            }

            return val;
        }

        public static bool Has<T>(this IEnumerable<T> list, Predicate<T> filter)
        {
            return list.FirstOrDefault(t => filter(t)) != null;
        }

        public static T CreateIfNull<T>(this T instance) where T : new()
        {
            if (instance == null)
                instance = new T();

            return instance;
        }

        public static C CreateOrClear<C, T>(this C collection) where C : ICollection<T>, new()
        {
            if (collection == null)
                collection = new C();
            else
                collection.Clear();

            return collection;
        }

        public static C CreateOrClear<C>(this C collection) where C : IList, new()
        {
            if (collection == null)
                collection = new C();
            else
                collection.Clear();

            return collection;
        }


        public static void ExpandeSize<T>(this List<T> list, int size)
        {
            while (list.Count < size)
            {
                list.Add(default(T));
            }
        }

        /// <summary>
        /// Copy the dictionary's content to another dictionary
        /// doesn't clear the 'to' dictionary
        /// </summary>
        /// <typeparam name="K"></typeparam>
        /// <typeparam name="V"></typeparam>
        /// <param name="from"></param>
        /// <param name="to"></param>
        public static void CopyDictionary<K, V>(this IDictionary<K, V> from, IDictionary<K, V> to)
        {
            foreach (KeyValuePair<K, V> kv in from)
            {
                to.Add(kv.Key, kv.Value);
            }
        }
    }
}
