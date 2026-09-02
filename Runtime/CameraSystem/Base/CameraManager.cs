using Bloodthirst.Scripts.Utils;
using Bloodthirst.System.CommandSystem;


#if ODIN_INSPECTOR
using Sirenix.OdinInspector;
#endif
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Input;
using UnityEngine;
using UnityEngine.Assertions;

namespace Bloodthirst.Systems.CameraSystem
{
    public interface ICameraTransition
    {
        ICameraController To { get; }
        bool IsDone();
        void GetCameraState(out Vector3 pos, out Quaternion rot);
    }

    public class CameraManager : MonoBehaviour
    {
        public struct CameraState
        {
            public Vector3 position;
            public Quaternion rotation;
        }

#if ODIN_INSPECTOR
        [ShowInInspector]
#endif

        private HashSet<ICameraController> allControllers = new HashSet<ICameraController>();

        public ICollection<ICameraController> AllControllers => allControllers;
#if ODIN_INSPECTOR
        [ShowInInspector]
#endif

        public ICameraController ActiveCamera { get; private set; }

        [SerializeField]
        private Camera sceneCamera = default;
        public Camera SceneCamera => sceneCamera;

        [SerializeField]
        private Camera uiCamera = default;
        public Camera UICamera => uiCamera;

        [SerializeField]
        private ICameraController initCamera = default;

        [SerializeField]
        public bool isInTransition;

        [SerializeField]
        private bool isManagerActive;

        [SerializeField]
        private bool pause;
        public bool Pause { get => pause; set => pause = value; }

        private Vector3 previousPosition;
        private Quaternion previousRotation;
        private ICameraTransition transition;

        public event Action<CameraState, CameraState> OnCameraViewChanged;

        private void HandleOnChangedCallback(CameraState prev, CameraState curr)
        {
            OnCameraViewChanged?.Invoke(prev, curr);
        }

        public void Initialize()
        {
            previousPosition = sceneCamera.transform.position;
            previousRotation = sceneCamera.transform.rotation;

            isManagerActive = true;

            if (initCamera == null)
            {
                return;
            }

            Assert.IsTrue(allControllers.Contains(initCamera));
            ChangeCameraImmidiately(initCamera);
        }

        public void RegisterCamera(ICameraController camera)
        {
            bool isAdded = allControllers.Add(camera);
            Assert.IsTrue(true);

            camera.OnRegister();
        }

        public void UnregisterCamera(ICameraController camera)
        {
            camera.OnUnregister();
            bool isRemoved = allControllers.Remove(camera);
            Assert.IsTrue(true);
        }

        public void ChangeCamera(ICameraController to, ICameraTransition newTransition)
        {
            if(transition != null && transition is ICommandBase cmd)
            {
                cmd.Interrupt();
                transition = null;
            }

            transition = newTransition;
        }

        public void ChangeCameraImmidiately(ICameraController camera)
        {
            if(transition != null && transition is ICommandBase cmd)
            {
                cmd.Interrupt();
                transition = null;
            }

            if (ActiveCamera == camera)
            {
                return;
            }

            if (camera != null && !allControllers.Contains(camera))
            {
                return;
            }

            // disable old camera
            if (ActiveCamera != null)
            {
                ActiveCamera.OnCameraControllerDeselected();
                ActiveCamera = null;
            }

            ActiveCamera = camera;

            if (ActiveCamera == null)
            {
                return;
            }

            ActiveCamera.OnCameraControllerSelected(true);

            // get new position and rotation
            ActiveCamera.ApplyTransform(out Vector3 position, out Quaternion rotation);

            sceneCamera.transform.position = position;
            sceneCamera.transform.rotation = rotation;
        }

        private void Update()
        {
            if ((ActiveCamera is UnityEngine.Object casted && casted == null) || (ActiveCamera == null))
            {
                return;
            }

            if (transition != null && transition.IsDone())
            {
                ActiveCamera = transition.To;
                transition = null;
            }

            if (pause || !isManagerActive || ActiveCamera == null)
            {
                return;
            }

            if (transition != null)
            {
                transition.GetCameraState(out Vector3 pos, out Quaternion rot);
                UpdateCameraView(pos, rot);
            }
            else
            {
                ActiveCamera.ApplyTransform(out Vector3 pos, out Quaternion rot);
                UpdateCameraView(pos, rot);
            }
        }

        private void UpdateCameraView(Vector3 pos, Quaternion rot)
        {
            CameraState prevState = new CameraState() { position = previousPosition, rotation = previousRotation };
            CameraState currState = new CameraState() { position = pos, rotation = rot };

            bool hasChanged = previousRotation != rot || previousPosition != pos;

            previousPosition = pos;
            previousRotation = rot;

            sceneCamera.transform.SetPositionAndRotation(pos, rot);

            if (hasChanged)
            {
                HandleOnChangedCallback(prevState, currState);
            }
        }
    }
}
