#if ODIN_INSPECTOR
	using Sirenix.OdinInspector;
#endif
using System;
using UnityEngine;

namespace Bloodthirst.Core.BISDSystem
{
    public abstract class EntityBehaviour<DATA, STATE, INSTANCE> : MonoBehaviour,

        IInitializeIdentifier,
        IInitializeInstance,

        IBehaviour,
        IBehaviourInstance<INSTANCE>,
        IBehaviourState<STATE>,
        IBehaviourData<DATA>,

        IBehaviourInstance,
        IBehaviourState,
        IBehaviourData,

        IPostEntitiesLoaded

        where DATA : EntityData
        where STATE : class, IEntityState<DATA>, new()
        where INSTANCE : EntityInstance<DATA, STATE, INSTANCE>, new()
    {

        private readonly Type stateType = typeof(STATE);
        private readonly Type instanceType = typeof(INSTANCE);
        private readonly Type dataType = typeof(DATA);

        [SerializeField]
        private bool initializeInstanceOnAwake;

        /// <summary>
        /// The parent entity containing this behaviour
        /// </summary>
        [SerializeField]
        [HideInInspector]
        private EntityIdentifier entityIdentifier;

        protected EntityIdentifier EntityIdentifier
        {
            get => entityIdentifier;
            set => entityIdentifier = value;
        }

        public DATA Data => instance.State.Data;

        public STATE State => instance.State;

        [SerializeField]
        private DATA tagData = default;

        public DATA TagData => tagData;

        #if ODIN_INSPECTOR[ShowInInspector]#endif
        protected INSTANCE instance = default;

        public INSTANCE Instance
        {
            get
            {
                return instance;
            }
            set
            {
                if (instance != null)
                {
                    instance.BeforeEntityRemoved -= OnEntityRemove;

                    OnDisposedInstance(instance);
                    instance.NotifyInstanceDisposed();
                }

                instance = value;

                if (instance == null)
                {
                    OnSetInstance(instance);
                    return;
                }

                instance.EntityIdentifier = EntityIdentifier;
                OnSetInstance(instance);

                instance.BeforeEntityRemoved -= OnEntityRemove;
                instance.BeforeEntityRemoved += OnEntityRemove;

                instance.NotifyInstanceBinded();

                instance.NotifyStateChanged();
            }
        }

        #region Unity callbacks
        protected virtual void Awake()
        {
            if(initializeInstanceOnAwake)
            {
                STATE state = new STATE();
                state.Data = tagData;

                INSTANCE ins = new INSTANCE();
                ins.State = state;

                Instance = ins;
            }
        }

        protected virtual void OnValidate()
        {
            if (entityIdentifier == null)
            {
                entityIdentifier = GetComponent<EntityIdentifier>();
            }
        }
        protected virtual void OnDestroy()
        {
            if (Instance == null)
                return;

            instance.BeforeEntityRemoved -= OnEntityRemove;
            instance = null;
        }
        #endregion

        /// <summary>
        /// <para>Inject the instance either by using the exterior data or by using the serialized instance</para>
        /// <para>This should only be called by the <see cref="EntitySpawner"/> and injection points</para>
        /// </summary>
        protected virtual void InitializeInstance(EntityIdentifier entityIdentifier)
        {
            // try loading from injector
            IInstanceInjector<INSTANCE> injector = GetComponentInChildren<IInstanceInjector<INSTANCE>>();

            if (injector != null)
            {
                Instance = injector.GetInstance();
                return;
            }

            // in case state or instance isn't serializable
            // create new ones
            if (Instance == null)
            {
                INSTANCE defaultInstance = new INSTANCE{};
                STATE state = new STATE();
                state.Data = TagData;
                state.InitDefaultState();

                defaultInstance.State = state;

                Instance = defaultInstance;
                return;
            }

            // if the state is null 
            // create one
            if (State == null)
            {
                STATE state = new STATE();
                state.Data = TagData;
                state.InitDefaultState();

                Instance.State = state;

                Instance = Instance;
                return;
            }

            Instance.State.InitDefaultState();
            Instance = Instance;
        }

        /// <summary>
        /// Method invoked after the instance has been changed
        /// </summary>
        /// <param name="instance"></param>
        public abstract void OnSetInstance(INSTANCE instance);

        /// <summary>
        /// <para>When the <see cref="Instance"/> is being changed , this method is invoked with the OLD instance passed as parameter</para>
        /// <para>Use this method to remove any links made to the old instance </para>
        /// </summary>
        /// <param name="instance"></param>
        public abstract void OnDisposedInstance(INSTANCE instance);

        /// <summary>
        /// <para>Gets invoked after the instance  calls for a remove </para>
        /// <para>Apply the removal that needs to be done on the behaviour part, like invoking Destroy or sending back to the pool</para>
        /// </summary>
        /// <param name="ins"></param>
        public virtual void OnEntityRemove(INSTANCE ins)
        {
            Instance = null;
        }

        /// <summary>
        /// <para>Method called by the entity spawner after all loading is done</para>
        /// <para>Usefull when we wanna look for a specific entity but we need to make sure that all the data is present in the scene</para>
        /// </summary>
        protected virtual void PostEntityLoaded() { }

        #region BISD interface implementation
        Type IBehaviourInstance.Type => instanceType;
        Type IBehaviourState.Type => stateType;
        Type IBehaviourData.Type => dataType;

        IEntityInstance IBehaviourInstance.Instance => Instance;
        IEntityState IBehaviourState.State => State;
        EntityData IBehaviourData.Data => Data;

        #endregion

        #region initialization interface implementations

        Type IInitializeInstance.StateType => stateType;

        void IInitializeInstance.InitializeInstance(EntityIdentifier entityIdentifier)
        {
            InitializeInstance(entityIdentifier);
        }

        void IInitializeIdentifier.InitializeIdentifier(EntityIdentifier entityIdentifier)
        {
            EntityIdentifier = entityIdentifier;
        }

        void IPostEntitiesLoaded.PostEntitiesLoaded()
        {
            PostEntityLoaded();
        }
        #endregion
    }
}
