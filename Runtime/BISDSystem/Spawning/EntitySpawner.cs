using Bloodthirst.Core.AdvancedPool.Pools;
using UnityEngine.Assertions;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Pool;

namespace Bloodthirst.Core.BISDSystem
{
    public class EntitySpawner
    {
        #region setup
        public void AssignID(GameObject entity)
        {
            // we set the id after since we might be creating new instances of the states
            EntityIdentifier id = entity.GetComponent<EntityIdentifier>();

            id.Id = EntityID.GetNextId();

            id.TriggerSpawned();
        }

        public void PostEntityLoaded(GameObject entity)
        {
            using (ListPool<IPostEntitiesLoaded>.Get(out var tmp))
            {
                entity.GetComponentsInChildren(true, tmp);

                foreach (IPostEntitiesLoaded cmp in tmp)
                {
                    cmp.PostEntitiesLoaded();
                }
            }
        }

        public static void IntializeInstances(EntityIdentifier id)
        {
            Assert.IsNotNull(id);

            using (ListPool<IInitializeInstance>.Get(out List<IInitializeInstance> tmp))
            {
                id.GetComponentsInChildren(true, tmp);

                // initialize identifier
                foreach (IInitializeInstance init in tmp)
                {
                    init.InitializeInstance(id);
                }
            }
        }

        public static void IntializeEntityIdentifier(EntityIdentifier id)
        {
            Assert.IsNotNull(id);

            using (ListPool<IInitializeIdentifier>.Get(out List<IInitializeIdentifier> tmp))
            {
                id.GetComponentsInChildren(true , tmp);

                // initialize identifier
                foreach (IInitializeIdentifier init in tmp)
                {
                    init.InitializeIdentifier(id);
                }
            }
        }

        public static void PostInitialize(EntityIdentifier id)
        {
            Assert.IsNotNull(id);

            using (ListPool<IEntityPostInit>.Get(out List<IEntityPostInit> tmp))
            {
                id.GetComponentsInChildren(true , tmp);

                // initialize identifier
                foreach (IEntityPostInit init in tmp)
                {
                    init.PostInit();
                }
            }
        }

        public static void InjectStates(GameObject entity)
        {
            // get instance register and provider and identifier

            EntityIdentifier id = entity.GetComponentInChildren<EntityIdentifier>();
            Assert.IsNotNull(id);

            IntializeEntityIdentifier(id);
            IntializeInstances(id);
            PostInitialize(id);
        }
        public BEHAVIOUR AddBehaviour<BEHAVIOUR, INSTANCE, STATE, DATA>(MonoBehaviour entity) where DATA : EntityData where STATE : class, IEntityState<DATA>, new() where INSTANCE : EntityInstance<DATA, STATE, INSTANCE>, new() where BEHAVIOUR : EntityBehaviour<DATA, STATE, INSTANCE>
        {
            // get instance register and provider and identifier

            EntityIdentifier entityIdentifier = entity.GetComponentInChildren<EntityIdentifier>();

            BEHAVIOUR behaviour = entity.gameObject.AddComponent<BEHAVIOUR>();

            ((IInitializeIdentifier)behaviour).InitializeIdentifier(entityIdentifier);

            // init instancee
            ((IInitializeInstance)behaviour).InitializeInstance(entityIdentifier);

            // query instance dependencies
            if (behaviour is IQueryInstance query)
            {
                query.QueryInstance();
            }

            return behaviour;
        }

        #endregion

        #region remove
        public void RemoveEntity(GameObject entity)
        {
            EntityIdentifier id = entity.GetComponentInChildren<EntityIdentifier>();
            Assert.IsNotNull(id);

            id.TriggerRemoved();
        }
        #endregion
    }
}
