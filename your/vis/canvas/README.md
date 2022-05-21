# Canvas
Canvas is an interface for rendering basic primitives. Compared to using OpenGL
directly, it has the following advantages:
- It supports both direct rendering (forwarding draw calls to OpenGL) and
  serialization (saving all the draw calls and data for later).
- It uses a uniform interface for both modes, allowing flexible switches.
- It makes common draw calls convenient to use (drawing a circle is a single
  call, unlike using OpenGL directly).

However using it for the direct rendering only has the following drawbacks:
- It does not support many OpenGL operations such as shading, texturing
  or FBO.
- It is very inefficient with many DrawXXX calls, as it cannot exploit the
  batch processing support from OpenGL (each DrawXXX call must be a separate
  pair of glBegin() and glEnd() calls).
