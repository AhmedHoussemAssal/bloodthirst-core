using System;
using System.Collections.Generic;
using UnityEngine;

namespace Bloodthirst.Core.AdvancedPool
{
   
    public class UnityPool<TObject> : UnityPoolBase where TObject : Component
    {
        private static readonly Type Type = typeof(TObject);
        public override Type PooledType => Type;

        private TObject Prefab;

        private int PoolCount { get; set; }

        private List<TObject> _pool;

        private List<TObject> _usedInstances;

        public List<TObject> UsedInstances => _UsedInstances;

        private List<TObject> _UsedInstances
        {
            get
            {
                if (_usedInstances == null)
                {
                    _usedInstances = new List<TObject>();
                }

                return _usedInstances;
            }
        }

        private Transform PoolTransform { get; set; }

        private List<TObject> _Pool
        {
            get
            {
                if (_pool == null)
                {
                    _pool = new List<TObject>();
                }

                return _pool;
            }
        }


        public UnityPool(Transform transform, TObject prefab, int poolCount)
        {
            PoolTransform = transform;
            Prefab = prefab;
            PoolCount = poolCount;
        }

        public void PopulatePool()
        {
            AddToPool(PoolCount);
        }

        private void AddToPool(int count)
        {
            for (int i = 0; i < count; i++)
            {
                TObject inst = UnityEngine.Object.Instantiate(Prefab, PoolTransform);
                inst.name = GetName(Prefab, i);
                inst.gameObject.SetActive(false);
                ReturnWithoutCallbacks(inst);
            }
        }

        public override void Return(GameObject t)
        {
            if (t == null)
            {
                Debug.Assert(t == null);
                return;
            }

            if (!t.TryGetComponent(out TObject comp))
            {
                Debug.LogError($"Instance doesn't contain contain a componenet of type {typeof(TObject).Name}");
            }

            IOnDespawn[] array = t.GetComponentsInChildren<IOnDespawn>(true);

            for (int i = 0; i < array.Length; i++)
            {
                IOnDespawn onSpawn = array[i];
                onSpawn.OnDespawn();
            }

            t.transform.SetParent(PoolTransform);
            t.gameObject.SetActive(false);
            _Pool.Add(comp);
            _UsedInstances.Remove(comp);
        }



        public TObject Get<T>(Predicate<TObject> filter)
        {
            CheckPoolSize();

            for (int i = 0; i < _Pool.Count; i++)
            {
                TObject ins = _Pool[i];

                if (!filter(ins))
                    continue;

                _Pool.RemoveAt(i);
                _UsedInstances.Add(ins);

                ins.gameObject.SetActive(true);

                IOnSpawn[] array = ins.GetComponentsInChildren<IOnSpawn>(true);
                for (int j = 0; j < array.Length; j++)
                {
                    IOnSpawn onSpawn = array[j];
                    onSpawn.OnSpawn();
                }

                return ins;
            }

            return null;

        }

        private void CheckPoolSize()
        {
            // re-instantiate if no more objects are available
            if (_Pool.Count == 0)
            {
                AddToPool(PoolCount);
            }
        }

        public override GameObject Get()
        {
            CheckPoolSize();

            TObject obj = _Pool[0];

            _Pool.RemoveAt(0);
            _UsedInstances.Add(obj);

            obj.transform.SetParent(null);
            obj.gameObject.SetActive(true);

            IOnSpawn[] array = obj.GetComponentsInChildren<IOnSpawn>(true);

            for (int i = 0; i < array.Length; i++)
            {
                IOnSpawn onSpawn = array[i];
                onSpawn.OnSpawn();
            }

            return obj.gameObject;
        }

        public TObject GetTyped()
        {
            CheckPoolSize();

            TObject obj = _Pool[0];

            _Pool.RemoveAt(0);
            _UsedInstances.Add(obj);

            obj.transform.SetParent(null);
            obj.gameObject.SetActive(true);

            IOnSpawn[] array = obj.GetComponentsInChildren<IOnSpawn>(true);

            for (int i = 0; i < array.Length; i++)
            {
                IOnSpawn onSpawn = array[i];
                onSpawn.OnSpawn();
            }

            return obj;
        }

        private void ReturnWithoutCallbacks(TObject t)
        {
            t.transform.SetParent(PoolTransform);
            t.gameObject.SetActive(false);
            _Pool.Add(t);
            _UsedInstances.Remove(t);
        }

        public void Return(TObject t)
        {
            if (t == null)
            {
                Debug.Assert(t == null);
                return;
            }

            IOnDespawn[] array = t.GetComponentsInChildren<IOnDespawn>(true);
            for (int i = 0; i < array.Length; i++)
            {
                IOnDespawn onSpawn = array[i];
                onSpawn.OnDespawn();
            }

            ReturnWithoutCallbacks(t);
        }

        protected override string GetName(UnityEngine.Object original, int index)
        {
            return $"{original.name} - {index}";
        }
    }
}