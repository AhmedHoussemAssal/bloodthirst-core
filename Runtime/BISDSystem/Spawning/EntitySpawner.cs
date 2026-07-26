using Bloodthirst.Core.AdvancedPool.Pools;
using UnityEngine.Assertions;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Pool;

namespace Bloodthirst.Core.BISDSystem
{
    public class EntitySpawner
    {
        public static void AssignID(GameObject entity)
        {
            // we set the id after since we might be creating new instances of the states
            EntityIdentifier id = entity.GetComponent<EntityIdentifier>();

            id.Id = EntityID.GetNextId();

            id.TriggerSpawned();
        }

        public static void PostEntityLoaded(GameObject entity)
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

        public static void PostInitialize(GameObject go)
        {
            using (ListPool<IEntityPostInit>.Get(out List<IEntityPostInit> tmp))
            {
                go.GetComponentsInChildren(true , tmp);

                // initialize identifier
                foreach (IEntityPostInit init in tmp)
                {
                    init.PostInit();
                }
            }
        }

        /// <summary>
        /// <para>Calls the entity initialization callbacks and propagates the entity ID into the child components</para>
        /// <para>NOTE : this assumes that the Entity ID is already correctly assigned</para>
        /// </summary>
        /// <param name="entity"></param>
        public static void InjectStates(GameObject entity)
        {
            // get instance register and provider and identifier

            EntityIdentifier id = entity.GetComponentInChildren<EntityIdentifier>();
            Assert.IsNotNull(id);

            IntializeEntityIdentifier(id);
            IntializeInstances(id);
            PostInitialize(id.gameObject);
        }
    }
}