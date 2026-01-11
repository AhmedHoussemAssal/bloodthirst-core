using Bloodthirst.Scripts.Utils;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using UnityEngine;

/// <summary>
/// An attempt to make an array optimized to deal with interating with a collection where we need a lot of add/remove ops
/// </summary>
public class InOutArray<T>
{
    #region core
    /// <summary>
    /// The actual array containing the elements
    /// </summary>
    private T[] InternalArray;

    /// <summary>
    /// A stack that supplies us with the recent empty gaps in the array
    /// </summary>
    private int[] EmptyIndiciesStack;

    private int stackPointer;

    /// <summary>
    /// <para>An index table used to jump to the correct action/method needed</para>
    /// <para>0 means do nothing</para>
    /// <para>1 means iterate</para>
    /// <para>Be careful when assigning the correct action to the correct index</para>
    /// </summary>
    private int[] flagArray;

    /// <summary>
    /// Pretty much the bounds of the array
    /// </summary>
    private int nextNewIndex;

    /// <summary>
    /// Number of elements present , not counting the empty gaps
    /// </summary>
    private int count;

    public int Count => count;

    /// <summary>
    /// How much of the internal buff are we using
    /// </summary>
    public int BufferSize => nextNewIndex;
    #endregion

    #region adding

    /// <summary>
    /// Array containing the appropiate Add Method to use/jump to
    /// </summary>
    private Func<T,int>[] addJumpTable;

    #endregion

    #region removing

    /// <summary>
    /// Array containing the appropiate Add Method to use/jump to
    /// </summary>
    private Action<int>[] removeAtJumpTable;

    #endregion

    #region iterating

    /// <summary>
    /// <para>0 means do nothing</para>
    /// <para>1 means iterate</para>
    /// <para>e careful when assigning the correct action to the correct index</para>
    /// </summary>
    private Action<T, int>[] iterateJumpTable;

    #endregion

    /// <summary>
    /// Initialize the Data structure with a max count of possible items to have at ounce
    /// </summary>
    /// <param name="initialSize">Max count of items</param>
    public InOutArray(int initialSize)
    {
        // just added a buffer value to that we can peek
        EmptyIndiciesStack = new int[initialSize];

        stackPointer = -1;

        // array
        InternalArray = new T[initialSize];

        // empty flags
        flagArray = new int[initialSize];

        for (int i = 0; i < initialSize; i++)
        {
            flagArray[i] = 0;
        }

        // last index to add in
        nextNewIndex = 0;
        count = 0;

        // add actions
        addJumpTable = new Func<T, int>[2];

        removeAtJumpTable = new Action<int>[2];

        iterateJumpTable = new Action<T, int>[2];

        addJumpTable[0] = AddAtTheEnd;
        addJumpTable[1] = InsertInGap;


        // remove at actions

        removeAtJumpTable[0] = RemoveAtEmpty;
        removeAtJumpTable[1] = RemoveAtExist;

        // this contains the actions done in the for loop
        // 0 means do nothing
        // 1 is where we plug the users callback
        iterateJumpTable[0] = IterateDoNothing;
        iterateJumpTable[1] = null;


    }

    #region adding

    // [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int Add(T item)
    {
        bool hasGap = stackPointer != -1;

        // 1 if have a gap we can insert into
        // 0 if we don't have a gap in the array
        int asInt = hasGap ? 1 : 0;

        return addJumpTable[asInt](item);
    }

    // [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int InsertInGap(T item)
    {
        int emptySlot = EmptyIndiciesStack[stackPointer--];

        flagArray[emptySlot] = 1;
        InternalArray[emptySlot] = item;
        count++;

        return emptySlot;
    }

    // [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int AddAtTheEnd(T item)
    {
        int insertionIndex = nextNewIndex++;
        flagArray[insertionIndex] = 1;
        InternalArray[insertionIndex] = item;
        count++;

        return insertionIndex;
    }

    #endregion


    #region removing

    // [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void RemoveAtEmpty(int index)
    {

    }

    // [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void RemoveAtExist(int index)
    {
        // we don't really need to remove , just add the index to the empty slots stack
        // and it will eventually be refilled in the next "Add"
        EmptyIndiciesStack[++stackPointer] = index;

        flagArray[index] = 0;
        count--;
    }

    // [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void RemoveAt(int index)
    {
        int jumpingIndex = flagArray[index];

        removeAtJumpTable[jumpingIndex](index);
    }

    #endregion


    #region iteration

    /// <summary>
    /// An iterator to help cleanly iterate over the array and avoid the gaps
    /// </summary>
    /// <param name="act"></param>
    // [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Iterate(Action<T, int> act)
    {
        iterateJumpTable[1] = act;

        for (int i = 0; i < BufferSize; i++)
        {
            int iterateIndex = flagArray[i];
            T item = InternalArray[i];
            iterateJumpTable[iterateIndex](item, i);
        }

    }

    public void GetAll(List<T> lst)
    {
        lst.Capacity = Mathf.Max(lst.Capacity, count);

        for (int i = 0; i < BufferSize; i++)
        {
            if (flagArray[i] != 1)
                continue;

            lst.Add(InternalArray[i]);
        }
    }

    // [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void IterateDoNothing(T item, int index)
    {

    }
    #endregion


    // [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Contains(T item)
    {
        bool found = false;

        int i = 0;

        while (i < BufferSize && !found)
        {
            T curr = InternalArray[i];

            found |= flagArray[i++] == 1 && curr.Equals(item);
        }

        return found;
    }

    // [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Clear()
    {
        int i = 0;

        while (i < count)
        {
            flagArray[i++] = 0;
        }

        nextNewIndex = 0;
        count = 0;
        stackPointer = -1;
    }
}