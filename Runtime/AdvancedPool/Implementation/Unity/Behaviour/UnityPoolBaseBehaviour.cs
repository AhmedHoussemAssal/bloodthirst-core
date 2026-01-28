#if ODIN_INSPECTOR || ODIN_INSPECTOR_3
	using Sirenix.OdinInspector;
#endif
using System;
using UnityEngine;

namespace Bloodthirst.Core.AdvancedPool
{
    public abstract class UnityPoolBaseBehaviour<TObject> : MonoBehaviour, IPoolBehaviour, IPoolBehaviour<TObject> where TObject : Component
    {
        private static readonly Type type = typeof(TObject);

        [SerializeField]
        protected Transform poolContainer = null;

        [SerializeField]
        protected TObject prefab;

        [SerializeField]
        protected int poolCount;

        public TObject Prefab
        {
            get => prefab;
            set => prefab = value;
        }

        GameObject IPoolBehaviour.Prefab
        {
            get => Prefab.gameObject;
            set => Prefab = value.GetComponent<TObject>();
        }


        int IPoolBehaviour.Count { get => poolCount; set => poolCount = value; }

        private UnityPool<TObject> _Pool;
        public UnityPool<TObject> Pool
        {
            get
            {
                if (_Pool == null)
                {
                    _Pool = new UnityPool<TObject>(poolContainer, prefab, poolCount);
                }
                return _Pool;
            }
        }

        Type IPoolBehaviour.Type => type;

        UnityPoolBase IPoolBehaviour.Pool => Pool;

        UnityPool<TObject> IPoolBehaviour<TObject>.PoolWithType => Pool;

        TObject IPoolBehaviour<TObject>.PrefabWithType => Prefab;

        void IPoolBehaviour.Initialize()
        {
            poolContainer = transform;
            _Pool = new UnityPool<TObject>(poolContainer, prefab, poolCount);
        }

        void IPoolBehaviour.Populate()
        {
            Pool.PopulatePool();
        }
    }
}
