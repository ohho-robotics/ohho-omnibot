package com.varunvaidhiya.robotcontrol.data.repository

import com.varunvaidhiya.robotcontrol.data.models.WheelSpeed
import com.varunvaidhiya.robotcontrol.network.ROSBridgeManager
import com.varunvaidhiya.robotcontrol.utils.Constants
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class RobotRepositoryTest {

    @Test
    fun `test wheel speed parsing`() {
        // Arrange
        val message = mapOf(
            "front_left" to 1.0,
            "front_right" to 2.0,
            "rear_left" to 3.0,
            "rear_right" to 4.0
        )
        
        val speeds = WheelSpeed(
            frontLeft = (message["front_left"] as Double).toFloat(),
            frontRight = (message["front_right"] as Double).toFloat(),
            backLeft = (message["rear_left"] as Double).toFloat(),
            backRight = (message["rear_right"] as Double).toFloat()
        )

        // Assert
        assertEquals(1.0f, speeds.frontLeft, 0.001f)
        assertEquals(2.0f, speeds.frontRight, 0.001f)
        assertEquals(3.0f, speeds.backLeft, 0.001f)
        assertEquals(4.0f, speeds.backRight, 0.001f)
    }

    @Test
    fun `arm and base teleop publish on the safety raw inputs`() {
        assertEquals("/cmd_vel/teleop_raw", Constants.TOPIC_CMD_VEL)
        assertEquals("/arm/joint_commands/teleop_raw", Constants.TOPIC_ARM_JOINT_COMMANDS)
        assertEquals("/ohho/heartbeat", Constants.TOPIC_HEARTBEAT)
        assertEquals(100L, Constants.HEARTBEAT_PERIOD_MS)
    }

    @Test
    fun `replaced manager callbacks do not stop the active heartbeat`() {
        val repository = RobotRepository()
        try {
            val replaced = repository.openManagerListener()
            val active = repository.openManagerListener()

            replaced.onConnected()
            assertFalse(repository.safetyHeartbeatActive)
            assertEquals(
                ROSBridgeManager.ConnectionState.DISCONNECTED,
                repository.connectionState.value
            )

            active.onConnected()
            assertTrue(repository.safetyHeartbeatActive)
            assertEquals(
                ROSBridgeManager.ConnectionState.CONNECTED,
                repository.connectionState.value
            )

            replaced.onDisconnected()
            replaced.onError("stale close")
            assertTrue(repository.safetyHeartbeatActive)
            assertEquals(
                ROSBridgeManager.ConnectionState.CONNECTED,
                repository.connectionState.value
            )

            active.onDisconnected()
            assertFalse(repository.safetyHeartbeatActive)
            assertEquals(
                ROSBridgeManager.ConnectionState.DISCONNECTED,
                repository.connectionState.value
            )
        } finally {
            repository.disconnect()
        }
    }

    @Test
    fun `disconnect ignores a late callback from the closed manager`() {
        val repository = RobotRepository()
        try {
            val listener = repository.openManagerListener()
            listener.onConnected()
            assertTrue(repository.safetyHeartbeatActive)

            repository.disconnect()
            assertFalse(repository.safetyHeartbeatActive)
            assertEquals(
                ROSBridgeManager.ConnectionState.DISCONNECTED,
                repository.connectionState.value
            )

            listener.onDisconnected()
            listener.onConnected()
            assertFalse(repository.safetyHeartbeatActive)
            assertEquals(
                ROSBridgeManager.ConnectionState.DISCONNECTED,
                repository.connectionState.value
            )
        } finally {
            repository.disconnect()
        }
    }

    @Test
    fun `current manager error stops the safety heartbeat`() {
        val repository = RobotRepository()
        try {
            val active = repository.openManagerListener()
            active.onConnected()
            assertTrue(repository.safetyHeartbeatActive)

            active.onError("socket failed")
            assertFalse(repository.safetyHeartbeatActive)
            assertEquals(
                ROSBridgeManager.ConnectionState.ERROR,
                repository.connectionState.value
            )
        } finally {
            repository.disconnect()
        }
    }
}
