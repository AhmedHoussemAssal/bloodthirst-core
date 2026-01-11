using Bloodthirst.Core.TreeList;
#if ODIN_INSPECTOR
	using Sirenix.Serialization;
#endif
using System;
using System.Collections.Generic;
using System.Linq;

namespace Bloodthirst.Core.BProvider
{
    public class BProvider
    {
        
#if ODIN_INSPECTOR
[OdinSerialize]
#endif

        private Dictionary<Type, InjectionInfo> typeToInjection;

        
#if ODIN_INSPECTOR
[OdinSerialize]
#endif

        private Dictionary<Type, TypeInfo> typeToInfoHash;

        
#if ODIN_INSPECTOR
[OdinSerialize]
#endif

        private TreeList<Type, IBProviderList> classInstances;

        
#if ODIN_INSPECTOR
[OdinSerialize]
#endif

        private TreeList<Type, IBProviderSingleton> classSingletons;

        
#if ODIN_INSPECTOR
[OdinSerialize]
#endif

        private TreeList<Type, IBProviderSingleton> interfaceSingletons;

        
#if ODIN_INSPECTOR
[OdinSerialize]
#endif

        private TreeList<Type, IBProviderList> interfaceInstances;

        internal Dictionary<Type, InjectionInfo> TypeToInjection
        {
            get => typeToInjection;
            set => typeToInjection = value;
        }

        /// <summary>
        /// Dictionary cache containing info about the types
        /// </summary>
        internal Dictionary<Type, TypeInfo> TypeToInfoHash
        {
            get => typeToInfoHash;
            set => typeToInfoHash = value;
        }

        #region class properties
        /// <summary>
        /// Tree structure to store the singletons
        /// </summary>
        internal TreeList<Type, IBProviderSingleton> ClassSingletons
        {
            get => classSingletons;
            set => classSingletons = value;
        }

        /// <summary>
        /// Tree structure to store the instances of concreate types
        /// </summary>
        internal TreeList<Type, IBProviderList> ClassInstances
        {
            get => classInstances;
            set => classInstances = value;
        }
        #endregion

        #region interface properties
        /// <summary>
        /// Tree structure to store the singletons
        /// </summary>
        internal TreeList<Type, IBProviderSingleton> InterfaceSingletons
        {
            get => interfaceSingletons;
            set => interfaceSingletons = value;
        }

        /// <summary>
        /// Tree structre to store interfaces
        /// </summary>
        internal TreeList<Type, IBProviderList> InterfaceInstances 
        { 
            get => interfaceInstances;
            set => interfaceInstances = value; 
        }
        #endregion

        public BProvider()
        {
            TypeToInfoHash = new Dictionary<Type, TypeInfo>();

            ClassInstances = new TreeList<Type, IBProviderList>();
            ClassSingletons = new TreeList<Type, IBProviderSingleton>();

            InterfaceInstances = new TreeList<Type, IBProviderList>();
            InterfaceSingletons = new TreeList<Type, IBProviderSingleton>();
        }

        #region singleton

        public object GetSingleton(Type t)
        {
            TypeInfo info = GetOrCreateInfo(t);

            if (info.SingletonLeaf == null)
            {
                info.SingletonLeaf = info.SingletonTree.GetOrCreateLeaf(info.TreeParentsList);
            }

            return info.SingletonLeaf?.Value?.Value;
        }

        public TSingletonType GetSingleton<TSingletonType>() where TSingletonType : class
        {
            return (TSingletonType)GetSingleton(typeof(TSingletonType));
        }

        public IEnumerable<TSingletonType> GetSingletons<TSingletonType>() where TSingletonType : class
        {
            Type t = typeof(TSingletonType);

            TypeInfo info = GetOrCreateInfo(t);

            if (info.SingletonLeaf == null)
            {
                info.SingletonLeaf = info.SingletonTree.GetOrCreateLeaf(info.TreeParentsList);
            }

            foreach (List<IBProviderSingleton> e in info.SingletonLeaf.TraverseAllSubElements())
            {
                for (int i = 0; i < e.Count; i++)
                {
                    yield return (TSingletonType)e[i].Value;
                }
            }
        }

        public bool RegisterOrReplaceSingleton<TInstanceType>(TInstanceType instance) where TInstanceType : class
        {
            return RegisterOrReplaceSingleton<TInstanceType, TInstanceType>(instance);
        }

        public bool RegisterOrReplaceSingleton<TInstanceType, ISingletonType>(TInstanceType instance) where ISingletonType : class where TInstanceType : ISingletonType
        {
            Type t = typeof(ISingletonType);

            TypeInfo info = GetOrCreateInfo(t);

            if (info.SingletonLeaf == null)
            {
                info.SingletonLeaf = info.SingletonTree.GetOrCreateLeaf(info.TreeParentsList);
            }

            if (info.SingletonLeaf.Value == null)
            {
                info.SingletonLeaf.Value = new BProviderSingleton<ISingletonType>(instance);
                return true;
            }

            info.SingletonLeaf.Value.Value = instance;

            return true;
        }

        public bool RegisterSingleton(Type t, object instance)
        {
            TypeInfo info = GetOrCreateInfo(t);

            if (info.SingletonLeaf == null)
            {
                info.SingletonLeaf = info.SingletonTree.GetOrCreateLeaf(info.TreeParentsList);
            }

            if (info.SingletonLeaf.Value == null)
            {
                info.SingletonLeaf.Value = CreateProviderSingleton(t, instance);
                return true;
            }

            // check if it's the same instance of not
            return info.SingletonLeaf.Value.Value == instance;
        }


        /// <summary>
        /// Register an instance 
        /// </summary>
        /// <typeparam name="TInstanceType"></typeparam>
        /// <param name="instance"></param>
        /// <returns></returns>
        public bool RegisterSingleton<TInstanceType, ISingletonType>(TInstanceType instance) where ISingletonType : class where TInstanceType : ISingletonType
        {
            Type t = typeof(ISingletonType);

            TypeInfo info = GetOrCreateInfo(t);

            if (info.SingletonLeaf == null)
            {
                info.SingletonLeaf = info.SingletonTree.GetOrCreateLeaf(info.TreeParentsList);
            }

            if (info.SingletonLeaf.Value == null)
            {
                info.SingletonLeaf.Value = new BProviderSingleton<ISingletonType>(instance);
                return true;
            }

            // check if it's the same instance of not
            return info.SingletonLeaf.Value.Value == (ISingletonType)instance;
        }

        /// <summary>
        /// Register an instance 
        /// </summary>
        /// <typeparam name="TInstanceType"></typeparam>
        /// <param name="instance"></param>
        /// <returns></returns>
        public bool RegisterSingleton<TInstanceType>(TInstanceType instance) where TInstanceType : class
        {
            return RegisterSingleton<TInstanceType, TInstanceType>(instance);
        }

        public bool RemoveSingleton(Type t, object instance)
        {
            TypeInfo info = GetOrCreateInfo(t);

            if (info.SingletonLeaf == null)
            {
                info.SingletonLeaf = info.SingletonTree.GetOrCreateLeaf(info.TreeParentsList);
            }

            if (info.SingletonLeaf.Value == null)
            {
                return false;
            }

            info.SingletonLeaf.Value = null;
            return true;
        }

        public bool RemoveSingleton<TInstanceType>(TInstanceType instance) where TInstanceType : class
        {
            Type t = typeof(TInstanceType);

            TypeInfo info = GetOrCreateInfo(t);

            if (info.SingletonLeaf == null)
            {
                info.SingletonLeaf = info.SingletonTree.GetOrCreateLeaf(info.TreeParentsList);
            }

            if (info.SingletonLeaf.Value == null)
            {
                return false;
            }

            info.SingletonLeaf.Value = null;
            return true;
        }


        #endregion

        #region multiple instances

        public IEnumerable<TInjectionType> GetInstances<TInjectionType>() where TInjectionType : class
        {
            Type t = typeof(TInjectionType);

            TypeInfo info = GetOrCreateInfo(t);

            if (info.InstanceLeaf == null)
            {
                info.InstanceLeaf = info.InstanceTree.GetOrCreateLeaf(info.TreeParentsList);
            }

            foreach (IBProviderList e in info.InstanceLeaf.TraverseAllSubElements())
            {
                foreach (object i in e.Elements)
                {
                    yield return (TInjectionType)i;
                }
            }
        }

        public bool RemoveInstance<TInstanceType>(TInstanceType instance) where TInstanceType : class
        {
            return RemoveInstance<TInstanceType, TInstanceType>(instance);
        }

        public bool RemoveInstance<TInstanceType, TInjectionType>(TInstanceType instance) where TInstanceType : class, TInjectionType
        {
            Type t = typeof(TInjectionType);

            TypeInfo info = GetOrCreateInfo(t);

            if (info.InstanceLeaf == null)
            {
                info.InstanceLeaf = info.InstanceTree.GetOrCreateLeaf(info.TreeParentsList);
            }

            if (info.InstanceLeaf.Value == null)
            {
                info.InstanceLeaf.Value = new BProviderList<TInjectionType>();
            }

            return info.InstanceLeaf.Value.Remove(instance);
        }

        public void RegisterInstance<TInstanceType>(TInstanceType instance) where TInstanceType : class
        {
            RegisterInstance<TInstanceType, TInstanceType>(instance);
        }

        public void RegisterInstance<TInstanceType, TInjectionType>(TInstanceType instance) where TInstanceType : class, TInjectionType
        {
            Type t = typeof(TInjectionType);

            TypeInfo info = GetOrCreateInfo(t);

            if (info.InstanceLeaf == null)
            {
                info.InstanceLeaf = info.InstanceTree.GetOrCreateLeaf(info.TreeParentsList);
            }

            if (info.InstanceLeaf.Value == null)
            {
                info.InstanceLeaf.Value = new BProviderList<TInjectionType>();
            }

            info.InstanceLeaf.Value.Add(instance);
        }

        public void RegisterInstance(Type t, object instance)
        {
            TypeInfo info = GetOrCreateInfo(t);

            if (info.InstanceLeaf == null)
            {
                info.InstanceLeaf = info.InstanceTree.GetOrCreateLeaf(info.TreeParentsList);
            }

            info.InstanceLeaf = info.InstanceTree.GetOrCreateLeaf(info.TreeParentsList);

            if (info.InstanceLeaf.Value == null)
            {
                info.InstanceLeaf.Value = CreateProviderList(t);
            }

            info.InstanceLeaf.Value.Add(instance);
        }

        public InstanceWatcher<T> GetInstanceWatcher<T>()
        {
            Type t = typeof(T);

            TypeInfo info = GetOrCreateInfo(t);

            if (info.InstanceLeaf == null)
            {
                info.InstanceLeaf = info.InstanceTree.GetOrCreateLeaf(info.TreeParentsList);
            }

            if (info.InstanceLeaf.Value == null)
            {
                info.InstanceLeaf.Value = new BProviderList<T>();
            }

            return new InstanceWatcher<T>(this);
        }

        public InstanceWatcher<T> GetSingletonWatcher<T>()
        {
            Type t = typeof(T);

            TypeInfo info = GetOrCreateInfo(t);

            if (info.SingletonLeaf == null)
            {
                info.SingletonLeaf = info.SingletonTree.GetOrCreateLeaf(info.TreeParentsList);
            }

            if (info.SingletonLeaf.Value == null)
            {
                info.SingletonLeaf.Value = new BProviderSingleton<T>(default);
            }

            return new InstanceWatcher<T>(this);
        }

        private IBProviderList CreateProviderList(Type t)
        {
            Type type = typeof(BProviderList<>).MakeGenericType(t);
            IBProviderList providerList = (IBProviderList)Activator.CreateInstance(type);
            return providerList;
        }
        private IBProviderSingleton CreateProviderSingleton(Type t, object value)
        {
            Type type = typeof(BProviderSingleton<>).MakeGenericType(t);
            return (IBProviderSingleton)Activator.CreateInstance(type, value);
        }

        #endregion

        #region common

        /// <summary>
        /// Get of create info about a certain type
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        private TypeInfo GetOrCreateInfo(Type t)
        {
            if (!TypeToInfoHash.TryGetValue(t, out TypeInfo info))
            {
                info = FlattenType(t);
                TypeToInfoHash.Add(t, info);
                return info;
            }

            return info;
        }

        /// <summary>
        /// Take a type and returns the flattened list of subtypes
        /// </summary>
        /// <param name="type"></param>
        /// <returns></returns>
        private TypeInfo FlattenType(Type type)
        {
            Type curr = type;

            bool isInsterface = curr.IsInterface;


            TreeList<Type, IBProviderList> instanceTree = null;
            TreeList<Type, IBProviderSingleton> singletonTree = null;

            switch (isInsterface)
            {
                case true:
                    instanceTree = InterfaceInstances;
                    singletonTree = InterfaceSingletons;
                    break;
                default:
                    instanceTree = ClassInstances;
                    singletonTree = ClassSingletons;
                    break;
            }

            List<Type> concreatSubTypes = new List<Type>();

            if (!isInsterface)
            {
                while (curr != null)
                {
                    concreatSubTypes.Insert(0, curr);
                    curr = curr.BaseType;
                }
            }

            else
            {
                concreatSubTypes.AddRange(type.GetInterfaces().ToList());
                concreatSubTypes.Insert(0 , curr);
            }

            TypeInfo typeInfo = new TypeInfo()
            {
                MainType = type,
                TreeParentsList = concreatSubTypes,
                InstanceTree = instanceTree,
                SingletonTree = singletonTree
            };

            return typeInfo;
        }

        #endregion

        public BProvider MergeWith(BProvider mergeWith)
        {
            foreach (KeyValuePair<Type, TypeInfo> kv in mergeWith.TypeToInfoHash)
            {
                if (TypeToInfoHash.ContainsKey(kv.Key))
                    continue;

                TypeToInfoHash.Add(kv.Key, kv.Value);
            }

            CopyTreeList(mergeWith.ClassInstances, ClassInstances);
            CopyTreeList(mergeWith.InterfaceInstances, InterfaceInstances);

            CopyTreeSingle(mergeWith.ClassSingletons, ClassSingletons);
            CopyTreeSingle(mergeWith.InterfaceSingletons, InterfaceSingletons);

            return this;
        }

        private void CopyTreeList(TreeList<Type, IBProviderList> src, TreeList<Type, IBProviderList> dest)
        {
            // copy classes
            List<TreeLeaf<Type, IBProviderList>> finals = src.GetFinalLeafs().ToList();

            foreach (TreeLeaf<Type, IBProviderList> f in finals)
            {
                List<Type> types = new List<Type>();

                TreeLeaf<Type, IBProviderList> firstMerge = f;
                TreeLeaf<Type, IBProviderList> curr = f;

                while (curr != null)
                {
                    types.Add(curr.LeafKey);

                    curr = curr.Parent;
                }

                TreeLeaf<Type, IBProviderList> firstThis = dest.GetOrCreateLeaf(types);

                while (firstMerge.Parent != null)
                {
                    if (firstMerge.Value != null)
                    {
                        if (firstThis.Value == null)
                        {
                            firstThis.Value = CreateProviderList(firstThis.LeafKey);
                        }

                        // add the content to the other node
                        foreach (object o in firstMerge.Value.Elements)
                        {
                            firstThis.Value.Add(o);
                        }
                    }

                    firstMerge = firstMerge.Parent;
                    firstThis = firstThis.Parent;
                }
            }
        }

        public void Inject(object instance)
        {

        }

        private void CopyTreeSingle(TreeList<Type, IBProviderSingleton> src, TreeList<Type, IBProviderSingleton> dest)
        {
            // copy classes
            List<TreeLeaf<Type, IBProviderSingleton>> finals = src.GetFinalLeafs().ToList();

            foreach (TreeLeaf<Type, IBProviderSingleton> f in finals)
            {
                List<Type> types = new List<Type>();

                TreeLeaf<Type, IBProviderSingleton> firstMerge = f;
                TreeLeaf<Type, IBProviderSingleton> curr = f;

                while (curr != null)
                {
                    types.Add(curr.LeafKey);

                    curr = curr.Parent;
                }

                TreeLeaf<Type, IBProviderSingleton> firstThis = dest.GetOrCreateLeaf(types);

                while (firstMerge.Parent != null)
                {
                    firstThis.Value = firstMerge.Value;

                    firstMerge = firstMerge.Parent;
                    firstThis = firstThis.Parent;
                }
            }
        }


    }
}