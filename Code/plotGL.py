import glfw
import OpenGL.GL as gl
from OpenGL.GL import shaders
import numpy as np
import pandas as pd
from math import sin, cos, radians

if not glfw.init():
    raise Exception("GLFW can't be initialized")

window = glfw.create_window(800, 600, "Glider Flight Simulator", None, None)
if not window:
    glfw.terminate()
    raise Exception("GLFW window can't be created")

glfw.make_context_current(window)


vertex_src = """
#version 330 core
layout(location = 0) in vec3 a_position;
void main() {
    gl_Position = vec4(a_position, 1.0);
}
"""
fragment_src = """
#version 330 core
out vec4 out_color;
void main() {
    out_color = vec4(0.0, 0.0, 1.0, 1.0);
}
"""

shader = compileProgram(
    compileShader(vertex_src, GL_VERTEX_SHADER),
    compileShader(fragment_src, GL_FRAGMENT_SHADER),
)


data = pd.read_csv("glider_data.csv")
time = data["time"].values
altitude = data["altitude"].values
pitch = np.deg2rad(data["pitch"].values)
yaw = np.deg2rad(data["yaw"].values)

# Convert pitch and yaw to a flight path
positions = []
x, y, z = 0, 0, altitude[0]
for i in range(1, len(time)):
    dx = cos(pitch[i]) * cos(yaw[i])
    dy = cos(pitch[i]) * sin(yaw[i])
    dz = sin(pitch[i])
    x += dx
    y += dy
    z += dz
    positions.append([x, y, z])
positions = np.array(positions, dtype=np.float32)

glEnable(GL_DEPTH_TEST)
while not glfw.window_should_close(window):
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glUseProgram(shader)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, positions)
    glEnableVertexAttribArray(0)
    glDrawArrays(GL_LINE_STRIP, 0, len(positions))

    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()
