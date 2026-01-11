#if ODIN_INSPECTOR || ODIN_INSPECTOR_3
#if ODIN_INSPECTOR
	using Sirenix.OdinInspector;
#endif
#endif
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;

namespace Bloodthirst.Core.Audio
{
    public class AudioManager : MonoBehaviour
    {
        [SerializeField]
        private bool isPreload = default;

        [SerializeField]
        private int preloadValue = default;

        private Queue<AudioSource> freeAudioSources = new Queue<AudioSource>();

        private List<AudioSource> busyAudioSources = new List<AudioSource>();

        private List<AudioSource> standaloneAudioSources = new List<AudioSource>();

        private AudioSource currentAudio;

        protected void Awake()
        {
            Clear();

            if (isPreload)
            {
                Preload();
            }
        }

#if ODIN_INSPECTOR || ODIN_INSPECTOR_3
        #if ODIN_INSPECTOR[Button]#endif
#endif
        private void Clear()
        {
            // clear standalone

            for (int i = standaloneAudioSources.Count - 1; i >= 0; i--)
            {
#if UNITY_EDITOR
                DestroyImmediate(standaloneAudioSources[i]);
#else
                Destroy(standaloneAudioSources[i]);
#endif
            }

            // clear busy

            for (int i = busyAudioSources.Count - 1; i >= 0; i--)
            {
#if UNITY_EDITOR
                DestroyImmediate(busyAudioSources[i]);
#else
                Destroy(busyAudioSources[i]);
#endif
            }

            // clear free

            while (freeAudioSources.Count != 0)
            {
#if UNITY_EDITOR
                DestroyImmediate(freeAudioSources.Dequeue());
#else
                Destroy(freeAudioSources.Dequeue());
#endif
            }
        }

        private void Preload()
        {
            for (int i = 0; i < preloadValue; i++)
            {
                GameObject go = new GameObject($"Pooled Audio Source");
                AudioSource audioSource = go.AddComponent<AudioSource>();

                ResetAudioSource(audioSource);

                freeAudioSources.Enqueue(audioSource);
            }
        }


        private void Update()
        {
            for (int i = busyAudioSources.Count - 1; i >= 0; i--)
            {
                currentAudio = busyAudioSources[i];

                // if the audio stopped

                if (!currentAudio.isPlaying && currentAudio.time >= currentAudio.clip.length)
                {
                    busyAudioSources.RemoveAt(i);

                    freeAudioSources.Enqueue(currentAudio);
                }
            }
        }

        private void ResetAudioSource(AudioSource audioSource)
        {
            audioSource.transform.SetParent(transform);
            audioSource.transform.SetLocalPositionAndRotation(Vector3.zero, Quaternion.identity);

            audioSource.spatialBlend = 0;
            audioSource.volume = 1;
            audioSource.loop = false;
            audioSource.outputAudioMixerGroup = null;
            audioSource.playOnAwake = false;
            audioSource.clip = null;
        }

        private AudioSource GetFreeAudioSource()
        {
            if (freeAudioSources.Count == 0)
            {
                GameObject go = new GameObject("Pooled Audio Source");
                AudioSource audioSource = go.AddComponent<AudioSource>();

                return audioSource;
            }

            return freeAudioSources.Dequeue();
        }
  
        public AudioSource GetStandaloneAudioSource()
        {
            AudioSource audioSource = GetFreeAudioSource();
            ResetAudioSource(audioSource);

            standaloneAudioSources.Add(audioSource);

            return audioSource;
        }

        public void ReturnStandaloneAudioSource(AudioSource audioSource)
        {
            bool removed = standaloneAudioSources.Remove(audioSource);
            Assert.IsTrue(removed);

            freeAudioSources.Enqueue(audioSource);
        }

#if ODIN_INSPECTOR        [Button]#endif
        public AudioSource PlayOnShot(AudioClip audioClip , Vector3 pos)
        {
            Assert.IsNotNull(audioClip);

            AudioSource audioSource = GetFreeAudioSource();
            ResetAudioSource(audioSource);

            audioSource.transform.position = pos;
            audioSource.clip = audioClip;
            audioSource.spatialBlend = 1;

            audioSource.Play();
            busyAudioSources.Add(audioSource);

            return audioSource;
        }

    }
}
