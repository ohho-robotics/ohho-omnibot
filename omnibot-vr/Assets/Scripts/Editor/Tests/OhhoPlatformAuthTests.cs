using System.IO;
using UnityEngine;
using Newtonsoft.Json;
using NUnit.Framework;
using OmniBot.VR.Core.Platform;

namespace OmniBot.VR.Tests
{
    /// <summary>
    /// Unit tests for where the headset gets its Supabase anon key: the bundled
    /// Resources/vr_manifest.json ships without one, and
    /// <see cref="OhhoPlatform.FillAuthFromLocal"/> fills it from the gitignored
    /// Resources/vr_manifest_local.json without overriding manifest values.
    /// </summary>
    public class OhhoPlatformAuthTests
    {
        private const string SiteUrl = "https://example.supabase.co";

        private static VrManifest ManifestWithoutKey() => new VrManifest
        {
            Auth = new VrAuthConfig
            {
                Provider = "supabase",
                Url = SiteUrl,
                AnonKey = "",
                SignIn = "email_otp",
                GarageTable = "user_robots",
            },
        };

        [Test]
        public void BundledManifest_HasUrlButNoAnonKey()
        {
            var path = Path.Combine(Application.dataPath, "Resources", "vr_manifest.json");
            var manifest = JsonConvert.DeserializeObject<VrManifest>(File.ReadAllText(path));
            Assert.IsNotNull(manifest.Auth);
            Assert.IsNotEmpty(manifest.Auth.Url);
            Assert.IsEmpty(manifest.Auth.AnonKey);
        }

        [Test]
        public void FillAuthFromLocal_EmptyKey_TakesLocalKey()
        {
            var manifest = ManifestWithoutKey();
            Assert.IsTrue(OhhoPlatform.FillAuthFromLocal(manifest, "{\"auth\": {\"anonKey\": \"local-key\"}}"));
            Assert.AreEqual("local-key", manifest.Auth.AnonKey);
            Assert.AreEqual(SiteUrl, manifest.Auth.Url);
            Assert.AreEqual("user_robots", manifest.Auth.GarageTable);
        }

        [Test]
        public void FillAuthFromLocal_ManifestValuesWin()
        {
            var manifest = ManifestWithoutKey();
            manifest.Auth.AnonKey = "site-key";
            Assert.IsTrue(OhhoPlatform.FillAuthFromLocal(manifest,
                "{\"auth\": {\"url\": \"https://other.supabase.co\", \"anonKey\": \"local-key\"}}"));
            Assert.AreEqual("site-key", manifest.Auth.AnonKey);
            Assert.AreEqual(SiteUrl, manifest.Auth.Url);
        }

        [Test]
        public void FillAuthFromLocal_ManifestWithoutAuth_GetsLocalAuth()
        {
            var manifest = new VrManifest();
            Assert.IsTrue(OhhoPlatform.FillAuthFromLocal(manifest,
                "{\"auth\": {\"url\": \"https://example.supabase.co\", \"anonKey\": \"local-key\"}}"));
            Assert.AreEqual(SiteUrl, manifest.Auth.Url);
            Assert.AreEqual("local-key", manifest.Auth.AnonKey);
        }

        [Test]
        public void FillAuthFromLocal_NoAuthBlock_LeavesKeyEmpty()
        {
            var manifest = ManifestWithoutKey();
            Assert.IsTrue(OhhoPlatform.FillAuthFromLocal(manifest, "{\"version\": 1}"));
            Assert.IsEmpty(manifest.Auth.AnonKey);
        }

        [Test]
        public void FillAuthFromLocal_MalformedJson_ReturnsFalseAndLeavesAuth()
        {
            var manifest = ManifestWithoutKey();
            Assert.IsFalse(OhhoPlatform.FillAuthFromLocal(manifest, "{\"auth\": {\"anonKey\": "));
            Assert.IsEmpty(manifest.Auth.AnonKey);
            Assert.AreEqual(SiteUrl, manifest.Auth.Url);
        }
    }
}
