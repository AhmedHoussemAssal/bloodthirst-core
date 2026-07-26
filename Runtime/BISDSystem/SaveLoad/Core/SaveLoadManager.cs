using Bloodthirst.Core.Utils;
#if ODIN_INSPECTOR
using Sirenix.OdinInspector;
#endif
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.Pool;

namespace Bloodthirst.Core.BISDSystem
{

    public struct SavedEntityEntry
    {
        public ISavableInstanceProvider instanceProvider;
        public List<ISaveState> states;
    }

    public class SaveLoadManager
    {
        private struct EntityStatePair
        {
            public GameObject entity;
            public IRuntimeState state;
        }

        private List<IGameStateLoader> loaders = new List<IGameStateLoader>();

        private List<IGameStateSaver> savers = new List<IGameStateSaver>();
        public IReadOnlyList<IGameStateLoader> Loaders => loaders;
        public IReadOnlyList<IGameStateSaver> Savers => savers;

        public void Initialize()
        {
            IEnumerable<Type> loadTypes = TypeUtils.AllTypes
                .Where(t => t.IsClass)
                .Where(t => !t.IsAbstract)
                .Where(t => TypeUtils.IsSubTypeOf(t, typeof(IGameStateLoader)));

            IEnumerable<Type> saveTypes = TypeUtils.AllTypes
                .Where(t => t.IsClass)
                .Where(t => !t.IsAbstract)
                .Where(t => TypeUtils.IsSubTypeOf(t, typeof(IGameStateSaver)));

            loaders.Clear();
            foreach (Type l in loadTypes)
            {
                IGameStateLoader loader = (IGameStateLoader)Activator.CreateInstance(l);
                loaders.Add(loader);
            }

            savers.Clear();
            foreach (Type s in saveTypes)
            {
                IGameStateSaver saver = (IGameStateSaver)Activator.CreateInstance(s);
                savers.Add(saver);
            }
        }


#if ODIN_INSPECTOR
        [Button]
#endif

        public void SaveRuntimeState(IReadOnlyList<GameObject> gos, List<SavedEntityEntry> results)
        {
            SavingContext context = new SavingContext();

            // get all savables
            using (ListPool<ISavableIdentifier>.Get(out List<ISavableIdentifier> ids))
            {
                GameObjectUtils.GetAllComponents(ref ids, gos, false);

                // get all the savables and organize themm into key/value pairs of instance -> states
                foreach (ISavableIdentifier id in ids)
                {
                    Component casted = (Component)id;
                    Assert.IsNotNull(casted);

                    // gameObject of the entity to save
                    GameObject saveableGo = casted.gameObject;

                    List<ISaveState> statesList = new List<ISaveState>();

                    // get the correct saver
                    foreach (IGameStateSaver saver in savers)
                    {
                        if (!saver.CanSave(saveableGo)) { continue; }

                        // create the gamesave from the state
                        ISaveState saveData = saver.GetSave(saveableGo, context);
                        Assert.IsNotNull(saveData);

                        statesList.Add(saveData);
                    }

                    // finally after all the states are saved for this entity
                    // we save them with the key being a class that contains all the necessary info to spawn or load
                    ISavableInstanceProvider instanceProvider = id.GetInstanceProvider();

                    results.Add(new SavedEntityEntry()
                    {
                        instanceProvider = instanceProvider,
                        states = statesList
                    });
                }
            }
        }

        public void SaveRuntimeState(List<SavedEntityEntry> results)
        {
            using (ListPool<GameObject>.Get(out var gos))
            {
                GameObjectUtils.GetAllRootGameObjects(gos);
                SaveRuntimeState(gos, results);
            }
        }

        public void LoadEntities(IReadOnlyList<SavedEntityEntry> savedEntities, Action<int, GameObject> onPreInitialize, Dictionary<SavedEntityEntry, GameObject> spawnedEntities, bool withPostLoad)
        {
            Assert.IsTrue(spawnedEntities.Count == 0);

            LoadingContext context = new LoadingContext();

            using (ListPool<IPostEntitiesLoaded>.Get(out List<IPostEntitiesLoaded> postLoads))
            using (DictionaryPool<SavedEntityEntry, EntityIdentifier>.Get(out Dictionary<SavedEntityEntry, EntityIdentifier> ids))
            using (DictionaryPool<IGameStateLoader, List<EntityStatePair>>.Get(out Dictionary<IGameStateLoader, List<EntityStatePair>> entityStatePairs))
            {
                // for each entity
                for (int i = 0; i < savedEntities.Count; i++)
                {
                    SavedEntityEntry kv = savedEntities[i];

                    GameObject spawned = kv.instanceProvider.GetInstanceToInject();
                    EntityIdentifier id = spawned.GetComponent<EntityIdentifier>();
                    Assert.IsNotNull(id);

                    onPreInitialize?.Invoke(i, spawned);

                    ids.Add(kv, id);

                    // set the entity identifier for all sub components
                    EntitySpawner.IntializeEntityIdentifier(id);

                    // ensure that all the sub components have an assigned instance
                    // with a default OR injected instance
                    EntitySpawner.IntializeInstances(id);

                    context.loadedEntities.Add(id);

                    // iterate over all the save states and inject them into the correct components
                    foreach (ISaveState saveState in kv.states)
                    {
                        // NOTE : notice how it's assumed that every saveState should be loaded by a UNIQUE loader
                        // if that for some reason changes , edit this part
                        IGameStateLoader loader = loaders.FirstOrDefault(l => l.CanLoad(spawned, saveState));

                        Assert.IsNotNull(loader, $"Couldn't find loader for the state of type {saveState.GetType()}");

                        IRuntimeState gameState = loader.ApplyState(spawned, saveState, context);

                        if (!entityStatePairs.TryGetValue(loader, out List<EntityStatePair> pairs))
                        {
                            pairs = new List<EntityStatePair>();
                            entityStatePairs.Add(loader, pairs);
                        }

                        // we store a list pairing every loader with a list of ALL the states it's responsible for loading
                        pairs.Add(new EntityStatePair() { entity = spawned, state = gameState });
                    }
                }

                // now , we use the loader-[states] entris to trigger the reference linking phase
                // each loader is responsible for linking it's list of specific states
                foreach (KeyValuePair<IGameStateLoader, List<EntityStatePair>> kv in entityStatePairs)
                {
                    IGameStateLoader loader = kv.Key;

                    foreach (var p in kv.Value)
                    {
                        loader.LinkReferences(p.entity, p.state, context);
                    }
                }

                // after all entities are loaded
                if (withPostLoad)
                {
                    foreach (EntityIdentifier id in ids.Values)
                    {
                        EntitySpawner.PostEntityLoaded(id.gameObject);
                    }
                }

                foreach (EntityIdentifier id in ids.Values)
                {
                    EntitySpawner.PostInitialize(id.gameObject);
                }

                foreach (KeyValuePair<SavedEntityEntry, EntityIdentifier> kv in ids)
                {
                    spawnedEntities.Add(kv.Key, kv.Value.gameObject);
                }
            }
        }
    }
}
