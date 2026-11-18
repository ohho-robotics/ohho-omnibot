using Newtonsoft.Json;

namespace OmniBot.VR.Core
{
    /// <summary>
    /// std_msgs/String — single string data field.
    /// </summary>
    public class StringMsg
    {
        [JsonProperty("data")]
        public string data;

        public StringMsg() { }

        public StringMsg(string data)
        {
            this.data = data;
        }
    }

    /// <summary>
    /// std_msgs/Bool — single boolean data field.
    /// </summary>
    public class BoolMsg
    {
        [JsonProperty("data")]
        public bool data;

        public BoolMsg() { }

        public BoolMsg(bool data)
        {
            this.data = data;
        }
    }

    /// <summary>
    /// std_msgs/Empty. No fields. Used for /ohho/heartbeat.
    /// </summary>
    public class EmptyMsg
    {
    }
}
