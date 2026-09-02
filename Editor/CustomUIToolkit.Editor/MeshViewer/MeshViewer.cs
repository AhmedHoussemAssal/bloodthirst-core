using UnityEditor;
using UnityEngine.UIElements;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Experimental.Rendering;
using Bloodthirst.Core.Utils;
using System;

namespace Bloodthirst.Editor.CustomComponent
{
    /// <summary>
    /// <para>A Custom UI Element that renders a mesh onto a texture and displays it as a UI Element</para>
    /// <para>The rendering can be controlled by passing a list of materials along with the camera's position and rotation</para>
    /// </summary>
    public class MeshViewer : VisualElement
    {

        private const string BASE_PATH = "Packages/com.bloodthirst.bloodthirst-core/Editor/CustomUIToolkit.Editor/" + nameof(MeshViewer);
        private const string UXML_PATH = BASE_PATH + "/" + nameof(MeshViewer) + ".uxml";
        private const string USS_PATH = BASE_PATH + "/" + nameof(MeshViewer) + ".uss";

        private Label _previewMessage => this.Q<Label>(nameof(_previewMessage));
        private Image _previewImage => this.Q<Image>(nameof(_previewImage));

        public Vector3 CameraPosition { get; set; }

        public float Yaw;
        public float Pitch;
        public Quaternion CameraRotation { get; set; }

        private CommandBuffer cmd;
        private RenderTexture _colorBuffer;
        private RenderTexture _depthBuffer;

        private UIInputListener _inputListener;

        private double _lastTimestamp;

        public Action<RenderTexture, RenderTexture, CommandBuffer> OnRenderCallback;

        public MeshViewer()
        {
            focusable = true;
            Focus();
            CameraRotation = Quaternion.identity;
            cmd = new CommandBuffer();
            cmd.name = "Editor rendering CommandBuffer";

            VisualTreeAsset visualTree = AssetDatabase.LoadAssetAtPath<VisualTreeAsset>(UXML_PATH);
            StyleSheet styling = AssetDatabase.LoadAssetAtPath<StyleSheet>(USS_PATH);

            visualTree.CloneTree(this);
            styleSheets.Add(EditorConsts.GlobalStyleSheet);
            styleSheets.Add(styling);

            RegisterCallback<AttachToPanelEvent>(HandleAdded);
            RegisterCallback<DetachFromPanelEvent>(HandleRemoved);

            _lastTimestamp = EditorApplication.timeSinceStartup;

            _inputListener = new UIInputListener(this);
        }

        private void HandleRemoved(DetachFromPanelEvent evt)
        {
            UnregisterCallback<AttachToPanelEvent>(HandleAdded);
            UnregisterCallback<DetachFromPanelEvent>(HandleRemoved);

            EditorApplication.update -= OnTick;

            _inputListener.Unbind();
        }

        private void HandleAdded(AttachToPanelEvent evt)
        {
            EditorApplication.update += OnTick;

            _inputListener.Bind();
        }

        private void OnTick()
        {
            double delta = EditorApplication.timeSinceStartup - _lastTimestamp;

            Input((float)delta);

            Render((float)delta);

            _lastTimestamp = EditorApplication.timeSinceStartup;

            _inputListener.PostTick();
        }

        private void Input(float delta)
        {
            UIInputListener.InputState inputState = _inputListener.State;

            // mouse input
            if (inputState.IsMousePressed)
            {
                int rotSpeed = 20;

                Yaw += inputState.MouseDelta.x * rotSpeed * delta;
                Pitch += inputState.MouseDelta.y * rotSpeed * delta;
                Pitch = Mathf.Clamp(Pitch, -70, 70);

                Quaternion rotUp = Quaternion.AngleAxis(Yaw, Vector3.up);
                Quaternion rotRight = rotUp * Quaternion.AngleAxis(Pitch, Vector3.right);
                Quaternion totalRot = rotUp * (rotUp * rotRight);

                Vector3 euler = totalRot.eulerAngles;

                Quaternion zCancel = Quaternion.AngleAxis(-euler.z, Vector3.forward);

                Quaternion rot = totalRot * zCancel;

                CameraRotation = rot;
            }

            // keyboard input
            Vector2 keyboard = Vector2.zero;

            {
                bool upPressed = inputState.KeyboardInput[(int)KeyCode.Z] == UIInputListener.KeyState.Pressed;
                bool downPressed = inputState.KeyboardInput[(int)KeyCode.S] == UIInputListener.KeyState.Pressed;
                bool rightPressed = inputState.KeyboardInput[(int)KeyCode.D] == UIInputListener.KeyState.Pressed;
                bool leftPressed = inputState.KeyboardInput[(int)KeyCode.Q] == UIInputListener.KeyState.Pressed;

                if (upPressed)
                {
                    keyboard.y += 1;
                }

                if (downPressed)
                {
                    keyboard.y -= 1;
                }

                if (rightPressed)
                {
                    keyboard.x += 1;
                }

                if (leftPressed)
                {
                    keyboard.x -= 1;
                }
            }

            float strafeSpeed = 2;
            float scrollSpeed = 2;

            Vector3 vel =
                (CameraRotation * Vector3.right * (keyboard.x * strafeSpeed)) +
                (CameraRotation * Vector3.up * (keyboard.y * strafeSpeed)) +
                (CameraRotation * Vector3.forward * (inputState.ScrollDelta * scrollSpeed));

            CameraPosition += vel * delta;
        }

        private void Render(float delta)
        {
            if (!_previewImage.IsLayoutBuilt())
            {
                _previewImage.image = _colorBuffer;
                _previewImage.MarkDirtyRepaint();
                return;
            }

            if (_previewImage.contentRect.width == 0 || _previewImage.contentRect.height == 0)
            {
                _previewImage.image = _colorBuffer;
                _previewImage.MarkDirtyRepaint();
                return;
            }

            bool recreatedTex = _colorBuffer == null ||
                            _colorBuffer.width != _previewImage.contentRect.width ||
                            _colorBuffer.height != _previewImage.contentRect.height;
            if (recreatedTex)
            {
                if (_colorBuffer != null)
                {
                    _colorBuffer.Release();
                }
                if (_depthBuffer != null)
                {
                    _depthBuffer.Release();
                }

                RenderTextureDescriptor desc = new RenderTextureDescriptor()
                {
                    width = (int)_previewImage.contentRect.width,
                    height = (int)_previewImage.contentRect.height,
                    depthBufferBits = 0,
                    msaaSamples = 1,
                    volumeDepth = 1,
                    depthStencilFormat = GraphicsFormat.None,
                    stencilFormat = GraphicsFormat.None,
                    enableRandomWrite = true,
                    dimension = TextureDimension.Tex2D,
                    graphicsFormat = GraphicsFormat.R8G8B8A8_SNorm
                };

                var colorDesc = desc;
                var depthDesc = desc;

                depthDesc.enableRandomWrite = false;
                depthDesc.depthBufferBits = 32;
                depthDesc.graphicsFormat = GraphicsFormat.None;
                depthDesc.depthStencilFormat = GraphicsFormat.D32_SFloat;
                _colorBuffer = new RenderTexture(colorDesc);
                _depthBuffer = new RenderTexture(depthDesc);
            }

            RenderToTexture(cmd);

            _previewImage.image = _colorBuffer;
            _previewImage.MarkDirtyRepaint();
        }

        private void RenderToTexture(CommandBuffer cmd)
        {
            float fov = 60;
            float aspect = _colorBuffer.width / (float)_colorBuffer.height;
            Matrix4x4 proj = Matrix4x4.Perspective(fov, aspect, 0.0001f, 100f);

            Matrix4x4 P = GL.GetGPUProjectionMatrix(proj, true);

            Matrix4x4 V = Matrix4x4.TRS(CameraPosition, CameraRotation, Vector3.one).inverse;

            Matrix4x4 M = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, Vector3.one);

            Matrix4x4 ZFlip = Matrix4x4.Scale(new Vector3(1, 1, -1));

            Matrix4x4 MVP = P * ZFlip * V * M;

            cmd.Clear();
            cmd.BeginSample("Editor rendering");
            cmd.SetRenderTarget(_colorBuffer, _depthBuffer);
            cmd.ClearRenderTarget(RTClearFlags.All, Color.clear, 1, 0);
            cmd.SetViewProjectionMatrices(ZFlip * V, P);

            OnRenderCallback?.Invoke(_colorBuffer, _depthBuffer, cmd);
            
            cmd.EndSample("Editor rendering");
            Graphics.ExecuteCommandBuffer(cmd);
        }


    }
}