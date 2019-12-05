// Define groups here so we can control the ordering.
/**
  @addtogroup geometry
  @{
    @defgroup geometry_roles Geometry Roles
    @defgroup render_engines Render Engines
  @}
*/

/** @addtogroup render_engines

 Perception simulation is supported via various rendering engines.

 Drake provides rendering capabilities via either `RenderEngineVtk` or
 `RenderEngineOspray` (as the ray-tracing engine within VTK). Depending on the
 intended usage, there are several trade-offs to consider when choosing between
 the render engines, such as features versus speed versus realism.

 __Features__

 The feature comparisons are summarised as follows:

 |             | VTK     | OSPRay  |
 |-------------|:-------:|:-------:|
 |RGB Image    | &diams; | &diams; |
 |Depth Image  | &diams; |         |
 |Label Image  | &diams; |         |
 |Shadows      |         | &diams; |
 |GPU          | &diams; |         |

 For more detail see MakeRenderEngineVtk() and MakeRenderEngineOspray().

 __Performance benchmarking__

 VTK produces faster results while OSPRay produces more realistic results.
 Speed differences between the two engines and their respective features
 were quantified by running Google Benchmark over variations on a simple scene
 of a sphere against a partial background. All times are in ms.

 |             | VTK              || OSPRay           ||
 |-------------|:-------:|:-------:|:-------:|:-------:|
 |             | Real    | CPU     | Real    | CPU     |
 |RGB Image    | 1.72676 | 1.43236 | 10.1692 | 10.1005 |
 |Depth Image  | 2.20887 | 2.17654 |         |         |
 |Label Image  | 2.42359 | 2.40398 |         |         |

 OSPRay's ray trace mode is configured to render with shadows turned on by
 default, though they can be disabled. This produces hard shadows and simple
 illumination effects at a relatively low computation cost. Alternatively, path
 tracing is also available for complex global illumination at a high computation
 cost. By default it is set to use one sample per pixel.

 |                          | OSPRay           ||
 |--------------------------|:-------:|:-------:|
 |                          | Real    | CPU     |
 |Ray trace with shadows    | 10.1692 | 10.1005 |
 |Ray trace without shadows | 10.0215 | 9.92181 |
 |Path trace (has shadows)  | 16.8442 | 16.708  |

 These results were obtained on a 56 core machine. To reproduce the benchmarking
 test locally, run `geometry/benchmarking/render_benchmark.cc` using:
 ```
 bazel run //geometry/benchmarking:render_benchmark
 ```

 */
