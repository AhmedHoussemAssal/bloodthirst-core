using Bloodthirst.Core.Utils;
using Sirenix.OdinInspector;
using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.EventSystems;
using UnityEngine.Pool;
using UnityEngine.UI;

namespace Bloodthirst.Core.UI
{
    public class UIButton : Selectable, IPointerClickHandler, ISubmitHandler , IPointerDownHandler
    {
        [SerializeField]
        private TriggerMode triggerMode;

        [SerializeField]
        private ButtonState state;
        public ButtonState State
        {
            get => state;
            set
            {
                if (state == value)
                    return;

                state = value;
                OnStateChanged?.Invoke(this);
            }
        }

        [SerializeField]
        private ClickEvent clickEvent;

        private Action<UIButton, ButtonState> clickEventCsharp;

        public event Action<UIButton, ButtonState> ClickEvent
        {
            add
            {
                clickEventCsharp += value;
                Assert.IsTrue(CheckDuplicateCallbacks());
            }
            remove
            {
                clickEventCsharp -= value;
                Assert.IsTrue(CheckDuplicateCallbacks());
            }
        }

        public event Action<UIButton> OnStateChanged;
        
        [ContextMenu("Simulate Click")]
        private void SimulateClick()
        {
            Assert.IsTrue(CheckDuplicateCallbacks());

            clickEvent.Invoke(this, state);
            clickEventCsharp?.Invoke(this, state);
        }

        public bool ShouldProcess(BaseEventData eventData)
        {
            using (ListPool<IUIEventFilter>.Get(out var filters))
            {
                GetComponents(filters);

                foreach (var f in filters)
                {
                    if (!f.ShouldProcess(eventData))
                    {
                        return false;
                    }
                }
            }

            return true;
        }

        void IPointerDownHandler.OnPointerDown(PointerEventData eventData)
        {
            if(triggerMode != TriggerMode.OnDown)
            {
                return;
            }

            if (!ShouldProcess(eventData))
            {
                return;
            }

            if (state != ButtonState.Enabled)
            {
                return;
            }

            Assert.IsTrue(CheckDuplicateCallbacks());

            clickEvent.Invoke(this, state);
            clickEventCsharp?.Invoke(this, state);
        }


        void IPointerClickHandler.OnPointerClick(PointerEventData eventData)
        {
            if (triggerMode != TriggerMode.Default)
            {
                return;
            }

            if (!ShouldProcess(eventData))
            {
                return;
            }

            if (state != ButtonState.Enabled)
            {
                return;
            }

            Assert.IsTrue(CheckDuplicateCallbacks());

            clickEvent.Invoke(this, state);
            clickEventCsharp?.Invoke(this, state);
        }

        private bool CheckDuplicateCallbacks()
        {
            bool isValid =
                !GameObjectUtils.HasDuplicateSubscriptions(clickEvent) &&
                !GameObjectUtils.HasDuplicateSubscriptions(clickEventCsharp);

            return isValid;
        }


        void ISubmitHandler.OnSubmit(BaseEventData eventData)
        {
            if (!ShouldProcess(eventData))
            {
                return;
            }

            if (state != ButtonState.Enabled)
            {
                return;
            }

            Assert.IsTrue(CheckDuplicateCallbacks());

            clickEvent.Invoke(this, state);
            clickEventCsharp?.Invoke(this, state);
        }
    }
}
