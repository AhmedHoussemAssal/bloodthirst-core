using Bloodthirst.Core.TreeList;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Bloodthirst.System.CommandSystem
{
    /// <summary>
    /// <para>Executes the subcommands sequentially using a queue order</para>
    /// <para>This doesn't account for the case when the sub-commands fail , whenever a subcommand fails it just gets dequeued a we got onto the next command</para>
    /// <para>For interruptable queue Look at <see cref="QueueInterruptableCommandBase{T}"/></para>
    /// </summary>
    /// <typeparam name="T"></typeparam>
    public abstract class TreeCommandBase<T> : CommandBase<T> where T : TreeCommandBase<T>
    {
        public event Action<ICommandBase, ICommandBase> OnCommandRemoved;
        public event Action<ICommandBase, ICommandBase> OnCommandAdded;

        private readonly bool propagateFailOrInterrupt;

        private List<TreeLeaf<int, CommandSettings>> cache;
        private int id;
        private TreeList<int, CommandSettings> RootNode { get; set; }

        private Func<int> cachedGetID;

        public TreeCommandBase(bool propagateFailOrInterrupt = false) : base()
        {
            this.propagateFailOrInterrupt = propagateFailOrInterrupt;
            RootNode = new TreeList<int, CommandSettings>();
            cachedGetID = IncrementID;

        }

        public override void OnStart()
        {
            cache = new List<TreeLeaf<int, CommandSettings>>();
            cache.AddRange(RootNode.GetAllLeafsDepthFirstFromFinalLeafsUp());
            cache.Reverse();
        }


        public CommandNodeBuilder Append(ICommandBase command, bool removeOnDone = true)
        {
            return Append(new CommandSettings() { Command = command }, removeOnDone);
        }

        private int IncrementID()
        {
            return ++id;
        }

        internal CommandNodeBuilder Append(CommandSettings commandSettings, bool removeOnDone = true)
        {
            int[] ids = new int[] { id++ };
            TreeLeaf<int, CommandSettings> firstLeaf = RootNode.GetOrCreateLeaf(ids);

            commandSettings.Command.RemoveWhenDone = removeOnDone;
            firstLeaf.Value = commandSettings;

            OnCommandAdded?.Invoke(this, commandSettings.Command);

            return new CommandNodeBuilder(firstLeaf, cachedGetID);
        }

        private void InterruptSubcommands()
        {
            foreach (TreeLeaf<int, CommandSettings> sub in RootNode.GetAllLeafsDepthFirst())
            {
                sub.Parent?.RemoveSubLeaf(sub);

                sub.Value.Command.GetExcutingCommand().Interrupt();

                OnCommandRemoved?.Invoke(this, sub.Value.Command);

            }
        }

        public override void OnTick(float delta)
        {
            while (cache.Count != 0)
            {
                TreeLeaf<int, CommandSettings> leaf = cache[cache.Count - 1];

                CommandSettings cmd = leaf.Value;

                ICommandBase currCmd = cmd.Command.GetExcutingCommand();

                // if command is not started , execute the command start
                if (currCmd.CommandState == COMMAND_STATE.WATING)
                {
                    currCmd.Start();
                }

                // if command is done , dequeue
                if (currCmd.IsDone())
                {
                    cache.RemoveAt(cache.Count - 1);

                    if (currCmd.CommandState == COMMAND_STATE.INTERRUPTED && propagateFailOrInterrupt)
                    {
                        Interrupt();
                        return;
                    }

                    if (currCmd.CommandState == COMMAND_STATE.FAILED && propagateFailOrInterrupt)
                    {
                        Fail();
                        return;
                    }

                    if (leaf.Parent != null)
                    {
                        leaf.Parent.RemoveSubLeaf(leaf);
                    }
                    else
                    {
                        RootNode.AllSubLeafs.Remove(leaf);
                    }

                    OnCommandRemoved?.Invoke(this, cmd.Command);
                    continue;
                }

                currCmd.OnTick(delta);
                return;
            }

            if (RemoveWhenDone)
            {
                Success();
                return;
            }
        }

        public override void OnEnd()
        {
            if (CommandState == COMMAND_STATE.INTERRUPTED)
            {
                InterruptSubcommands();
            }
        }

    }
}
