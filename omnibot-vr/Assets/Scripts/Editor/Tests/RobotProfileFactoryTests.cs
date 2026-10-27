using NUnit.Framework;
using OmniBot.VR.Control;
using OmniBot.VR.Core;
using OmniBot.VR.Core.Platform;

namespace OmniBot.VR.Tests
{
    /// <summary>
    /// OmniBot's headset arm commands must use the mux teleop input.
    /// arm_cmd_mux holds /arm/joint_commands (the policy alias) in teleop.
    /// </summary>
    public class RobotProfileFactoryTests
    {
        private static GarageRobot OmniBot(string modelId)
        {
            return new GarageRobot
            {
                UserRobot = new UserRobot { Id = "user-robot", Name = "OmniBot" },
                Category = new VrCategory { Id = "mobile-manipulator", Label = "Mobile manipulator" },
                RobotType = new VrRobotType
                {
                    Id = "omnibot-type",
                    Name = "OmniBot",
                    Category = "mobile-manipulator",
                },
                HardwareModel = new VrHardwareModel
                {
                    Id = modelId,
                    Name = "OmniBot",
                    Locomotion = "mecanum holonomic",
                    HasArm = true,
                },
            };
        }

        [Test]
        public void OmniBotProfile_PublishesArmCommandsOnTeleopTopic()
        {
            var profile = RobotProfileFactory.FromGarageRobot(OmniBot("omnibot"));

            Assert.IsNotNull(profile);
            Assert.AreEqual("/arm/joint_commands/teleop", profile.Topics.ArmCommands);
            Assert.AreEqual(RobotConfig.TOPIC_ARM_COMMANDS, profile.Topics.ArmCommands);
        }

        [Test]
        public void OmniBotProProfile_PublishesArmCommandsOnTeleopTopic()
        {
            var profile = RobotProfileFactory.FromGarageRobot(OmniBot("omnibot-pro"));

            Assert.IsNotNull(profile);
            Assert.AreEqual("/arm/joint_commands/teleop", profile.Topics.ArmCommands);
        }
    }
}
