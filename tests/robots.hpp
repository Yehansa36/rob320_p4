#pragma once

const char *SIMPLEBOT_JRDF = R"(
{
  "name": "SimpleBot",
  "links": [
    {
      "name": "base",
      "visuals": [
        {
          "geometry": {
            "type": "box",
            "size": [0.1, 0.1, 0.05]
          },
          "material": {
            "name": "",
            "color": [0.0, 0.15294, 0.29804, 1.0]
          },
          "origin": {
            "xyz": [-0.05, -0.05, 0],
            "rpy": [0, 0, 0]
          }
        }
      ]
    },
    {
      "name": "torso",
      "visuals": [
        {
          "geometry": {
            "type": "box",
            "size": [0.075, 0.075, 0.4]
          },
          "material": {
            "name": "",
            "color": [1.0, 0.79608, 0.01961, 1.0]
          },
          "origin": {
            "xyz": [-0.0375, -0.0375, 0.05],
            "rpy": [0, 0, 0]
          }
        }
      ]
    },
    {
      "name": "upper_arm",
      "visuals": [
        {
          "geometry": {
            "type": "box",
            "size": [0.2, 0.02, 0.02]
          },
          "material": {
            "name": "",
            "color": [0.0, 0.15294, 0.29804, 1.0]
          },
          "origin": {
            "xyz": [0, -0.01, 0],
            "rpy": [0, 0, 0]
          }
        }
      ]
    },
    {
      "name": "forearm",
      "visuals": [
        {
          "geometry": {
            "type": "box",
            "size": [0.1, 0.02, 0.02]
          },
          "material": {
            "name": "",
            "color": [0.0, 0.15294, 0.29804, 1.0]
          },
          "origin": {
            "xyz": [0, -0.01, 0],
            "rpy": [0, 0, 0]
          }
        }
      ]
    },
    {
      "name": "tool",
      "visuals": [
        {
          "geometry": {
            "type": "cylinder",
            "length": 0.025,
            "radius": 0.005
          },
          "material": {
            "name": "",
            "color": [1.0, 0.79608, 0.01961, 1.0]
          },
          "origin": {
            "xyz": [0, 0, 0.01],
            "rpy": [0, 1.5708, 0]
          }
        }
      ]
    },
    {
      "name": "eyeball_left",
      "visuals": [
        {
          "geometry": {
            "type": "sphere",
            "radius": 0.02
          },
          "material": {
            "name": "",
            "color": [1.0, 1.0, 1.0, 1.0]
          }
        }
      ]
    },
    {
      "name": "eyeball_right",
      "visuals": [
        {
          "geometry": {
            "type": "sphere",
            "radius": 0.02
          },
          "material": {
            "name": "",
            "color": [1.0, 1.0, 1.0, 1.0]
          }
        }
      ]
    },
    {
      "name": "pupil_left",
      "visuals": [
        {
          "geometry": {
            "type": "sphere",
            "radius": 0.01
          },
          "material": {
            "name": "",
            "color": [0.0, 0.0, 0.0, 1.0]
          },
          "origin": {
            "xyz": [0, -0.015, 0],
            "rpy": [0, 0, 0]
          }
        }
      ]
    },
    {
      "name": "pupil_right",
      "visuals": [
        {
          "geometry": {
            "type": "sphere",
            "radius": 0.01
          },
          "material": {
            "name": "",
            "color": [0.0, 0.0, 0.0, 1.0]
          },
          "origin": {
            "xyz": [0, -0.015, 0],
            "rpy": [0, 0, 0]
          }
        }
      ]
    }
  ],
  "joints": [
    {
      "name": "waist",
      "parent": "base",
      "child": "torso",
      "type": "revolute",
      "axis": [0, 0, 1],
      "limits": {
        "velocity": 100.0,
        "effort": 1000.0,
        "lower": -3.1415,
        "upper": 3.1415
      }
    },
    {
      "name": "shoulder",
      "parent": "torso",
      "child": "upper_arm",
      "type": "revolute",
      "axis": [0, 1, 0],
      "origin": {
        "xyz": [0.0375, 0, 0.3],
        "rpy": [0, 0, 0]
      },
      "limits": {
        "velocity": 100.0,
        "effort": 1000.0,
        "lower": -1.5708,
        "upper": 1.5708
      }
    },
    {
      "name": "elbow",
      "parent": "upper_arm",
      "child": "forearm",
      "type": "revolute",
      "axis": [0, 1, 0],
      "origin": {
        "xyz": [0.2, 0, 0],
        "rpy": [0, 0, 0]
      },
      "limits": {
        "velocity": 100.0,
        "effort": 1000.0,
        "lower": -1.5708,
        "upper": 1.5708
      }
    },
    {
      "name": "wrist",
      "parent": "forearm",
      "child": "tool",
      "type": "prismatic",
      "axis": [0, 1, 0],
      "origin": {
        "xyz": [0.1, 0, 0],
        "rpy": [0, 0, 0]
      },
      "limits": {
        "velocity": 100.0,
        "effort": 1000.0,
        "lower": 0,
        "upper": 0.0125
      }
    },
    {
      "name": "eye_socket_left",
      "parent": "torso",
      "child": "eyeball_left",
      "type": "fixed",
      "origin": {
        "xyz": [0.0375, -0.0375, 0.45],
        "rpy": [0, 0, 0]
      }
    },
    {
      "name": "eye_socket_right",
      "parent": "torso",
      "child": "eyeball_right",
      "type": "fixed",
      "origin": {
        "xyz": [-0.0375, -0.0375, 0.45],
        "rpy": [0, 0, 0]
      }
    },
    {
      "name": "eye_lens_left",
      "parent": "eyeball_left",
      "child": "pupil_left",
      "type": "revolute",
      "axis": [0, 0, 1],
      "limits": {
        "velocity": 100.0,
        "effort": 1000.0,
        "lower": -1,
        "upper": 1
      }
    },
    {
      "name": "eye_lens_right",
      "parent": "eyeball_right",
      "child": "pupil_right",
      "type": "revolute",
      "axis": [1, 0, 0],
      "limits": {
        "velocity": 100.0,
        "effort": 1000.0,
        "lower": -1,
        "upper": 1
      }
    }
  ]
}
)";

const char *RX200_JRDF = R"(
{
  "name": "ReactorX-200",
  "links": [
    {
      "name": "/base_link",
      "visuals": [
        {
          "geometry": {
            "type": "mesh",
            "filename": "rx200/RXA-200-M-1-Base.stl",
            "scale": [
              0.001,
              0.001,
              0.001
            ]
          },
          "origin": {
            "xyz": [
              0.0,
              0.0,
              0.0
            ],
            "rpy": [
              0.0,
              0.0,
              1.5707963267948966
            ]
          },
          "material": {
            "name": "interbotix_black"
          }
        }
      ],
      "collisions": [
        {
          "geometry": {
            "type": "mesh",
            "filename": "rx200/RXA-200-M-1-Base.stl",
            "scale": [
              0.001,
              0.001,
              0.001
            ]
          },
          "origin": {
            "xyz": [
              0.0,
              0.0,
              0.0
            ],
            "rpy": [
              0.0,
              0.0,
              -1.5707963267948966
            ]
          },
          "material": {
            "name": "interbotix_black"
          }
        }
      ],
      "inertial": {
        "origin": {
          "xyz": [
            -0.0354389,
            -0.00335861,
            0.0253408
          ],
          "rpy": [
            0.0,
            0.0,
            1.5707963267948966
          ]
        },
        "mass": 0.766135,
        "inertia": {
          "ixx": 0.003353,
          "ixy": -0.0001246,
          "ixz": 1.106e-05,
          "iyy": 0.001357,
          "iyz": 0.0002103,
          "izz": 0.004111
        }
      }
    },
    {
      "name": "/shoulder_link",
      "visuals": [
        {
          "geometry": {
            "type": "mesh",
            "filename": "rx200/RXA-200-M-2-Shoulder.stl",
            "scale": [
              0.001,
              0.001,
              0.001
            ]
          },
          "origin": {
            "xyz": [
              0.0,
              0.0,
              0.0015
            ],
            "rpy": [
              0.0,
              0.0,
              -1.5707963267948966
            ]
          },
          "material": {
            "name": "interbotix_black"
          }
        }
      ],
      "collisions": [
        {
          "geometry": {
            "type": "mesh",
            "filename": "rx200/RXA-200-M-2-Shoulder.stl",
            "scale": [
              0.001,
              0.001,
              0.001
            ]
          },
          "origin": {
            "xyz": [
              0.0,
              0.0,
              0.0015
            ],
            "rpy": [
              0.0,
              0.0,
              -1.5707963267948966
            ]
          },
          "material": {
            "name": "interbotix_black"
          }
        }
      ],
      "inertial": {
        "origin": {
          "xyz": [
            -8.53644e-05,
            1.7369e-05,
            0.0132005
          ],
          "rpy": [
            0.0,
            0.0,
            1.5707963267948966
          ]
        },
        "mass": 0.257774,
        "inertia": {
          "ixx": 0.0002663,
          "ixy": 9e-10,
          "ixz": 5.11e-08,
          "iyy": 0.0004428,
          "iyz": 4.416e-07,
          "izz": 0.0004711
        }
      }
    },
    {
      "name": "/upper_arm_link",
      "visuals": [
        {
          "geometry": {
            "type": "mesh",
            "filename": "rx200/RXA-200-M-3-UA.stl",
            "scale": [
              0.001,
              0.001,
              0.001
            ]
          },
          "origin": {
            "xyz": [
              0.0,
              0.0,
              0.0
            ],
            "rpy": [
              0.0,
              0.0,
              1.5707963267948966
            ]
          },
          "material": {
            "name": "interbotix_black"
          }
        }
      ],
      "collisions": [
        {
          "geometry": {
            "type": "mesh",
            "filename": "rx200/RXA-200-M-3-UA.stl",
            "scale": [
              0.001,
              0.001,
              0.001
            ]
          },
          "origin": {
            "xyz": [
              0.0,
              0.0,
              0.0
            ],
            "rpy": [
              0.0,
              0.0,
              1.5707963267948966
            ]
          },
          "material": {
            "name": "interbotix_black"
          }
        }
      ],
      "inertial": {
        "origin": {
          "xyz": [
            0.0119513,
            -0.000116923,
            0.13943
          ],
          "rpy": [
            0.0,
            0.0,
            1.5707963267948966
          ]
        },
        "mass": 0.297782,
        "inertia": {
          "ixx": 0.00171,
          "ixy": -9.773e-07,
          "ixz": 2.0936e-06,
          "iyy": 0.001631,
          "iyz": 0.0002132,
          "izz": 0.0001478
        }
      }
    },
    {
      "name": "/forearm_link",
      "visuals": [
        {
          "geometry": {
            "type": "mesh",
            "filename": "rx200/RXA-200-M-4-Forearm.stl",
            "scale": [
              0.001,
              0.001,
              0.001
            ]
          },
          "origin": {
            "xyz": [
              0.0,
              0.0,
              0.0
            ],
            "rpy": [
              0.0,
              0.0,
              1.5707963267948966
            ]
          },
          "material": {
            "name": "interbotix_black"
          }
        }
      ],
      "collisions": [
        {
          "geometry": {
            "type": "mesh",
            "filename": "rx200/RXA-200-M-4-Forearm.stl",
            "scale": [
              0.001,
              0.001,
              0.001
            ]
          },
          "origin": {
            "xyz": [
              0.0,
              0.0,
              0.0
            ],
            "rpy": [
              0.0,
              0.0,
              1.5707963267948966
            ]
          },
          "material": {
            "name": "interbotix_black"
          }
        }
      ],
      "inertial": {
        "origin": {
          "xyz": [
            0.114745,
            -9.38376e-05,
            0.0
          ],
          "rpy": [
            0.0,
            0.0,
            1.5707963267948966
          ]
        },
        "mass": 0.258863,
        "inertia": {
          "ixx": 0.001055,
          "ixy": -1.8286e-06,
          "ixz": 0.0,
          "iyy": 6.421e-05,
          "iyz": 0.0,
          "izz": 0.001076
        }
      }
    },
    {
      "name": "/wrist_link",
      "visuals": [
        {
          "geometry": {
            "type": "mesh",
            "filename": "rx200/RXA-200-M-5-Wrist.stl",
            "scale": [
              0.001,
              0.001,
              0.001
            ]
          },
          "origin": {
            "xyz": [
              0.0,
              0.0,
              0.0
            ],
            "rpy": [
              0.0,
              3.141592653589793,
              -1.5707963267948966
            ]
          },
          "material": {
            "name": "interbotix_black"
          }
        }
      ],
      "collisions": [
        {
          "geometry": {
            "type": "mesh",
            "filename": "rx200/RXA-200-M-5-Wrist.stl",
            "scale": [
              0.001,
              0.001,
              0.001
            ]
          },
          "origin": {
            "xyz": [
              0.0,
              0.0,
              0.0
            ],
            "rpy": [
              0.0,
              3.141592653589793,
              -1.5707963267948966
            ]
          },
          "material": {
            "name": "interbotix_black"
          }
        }
      ],
      "inertial": {
        "origin": {
          "xyz": [
            0.04236,
            1.0411e-05,
            -0.010577
          ],
          "rpy": [
            0.0,
            3.141592653589793,
            -1.5707963267948966
          ]
        },
        "mass": 0.084957,
        "inertia": {
          "ixx": 3.082e-05,
          "ixy": 1.91e-08,
          "ixz": 2.3e-09,
          "iyy": 2.822e-05,
          "iyz": 2.5481e-06,
          "izz": 3.152e-05
        }
      }
    },
    {
      "name": "/gripper_link",
      "visuals": [
        {
          "geometry": {
            "type": "mesh",
            "filename": "rx200/RXA-200-M-6-Gripper.stl",
            "scale": [
              0.001,
              0.001,
              0.001
            ]
          },
          "origin": {
            "xyz": [
              -0.02,
              0.0,
              0.0
            ],
            "rpy": [
              0.0,
              0.0,
              1.5707963267948966
            ]
          },
          "material": {
            "name": "interbotix_black"
          }
        }
      ],
      "collisions": [
        {
          "geometry": {
            "type": "mesh",
            "filename": "rx200/RXA-200-M-6-Gripper.stl",
            "scale": [
              0.001,
              0.001,
              0.001
            ]
          },
          "origin": {
            "xyz": [
              -0.02,
              0.0,
              0.0
            ],
            "rpy": [
              0.0,
              0.0,
              1.5707963267948966
            ]
          },
          "material": {
            "name": "interbotix_black"
          }
        }
      ],
      "inertial": {
        "origin": {
          "xyz": [
            0.02163,
            0.0,
            0.01141
          ],
          "rpy": [
            0.0,
            0.0,
            1.5707963267948966
          ]
        },
        "mass": 0.072885,
        "inertia": {
          "ixx": 2.537e-05,
          "ixy": 0.0,
          "ixz": 0.0,
          "iyy": 1.836e-05,
          "iyz": 4.34e-07,
          "izz": 1.674e-05
        }
      }
    },
    {
      "name": "/ee_arm_link",
      "inertial": {
        "origin": {
          "xyz": [
            0,
            0,
            0
          ],
          "rpy": [
            0,
            0,
            0
          ]
        },
        "mass": 0.001,
        "inertia": {
          "ixx": 0.001,
          "ixy": 0.0,
          "ixz": 0.0,
          "iyy": 0.001,
          "iyz": 0.0,
          "izz": 0.001
        }
      }
    },
    {
      "name": "/gripper_prop_link",
      "visuals": [
        {
          "geometry": {
            "type": "mesh",
            "filename": "rx200/RXA-200-M-7-Gripper-Prop.stl",
            "scale": [
              0.001,
              0.001,
              0.001
            ]
          },
          "origin": {
            "xyz": [
              -0.0685,
              0.0,
              0.0
            ],
            "rpy": [
              0.0,
              0.0,
              1.5707963267948966
            ]
          },
          "material": {
            "name": "interbotix_black"
          }
        }
      ],
      "collisions": [
        {
          "geometry": {
            "type": "mesh",
            "filename": "rx200/RXA-200-M-7-Gripper-Prop.stl",
            "scale": [
              0.001,
              0.001,
              0.001
            ]
          },
          "origin": {
            "xyz": [
              -0.0685,
              0.0,
              0.0
            ],
            "rpy": [
              0.0,
              0.0,
              1.5707963267948966
            ]
          },
          "material": {
            "name": "interbotix_black"
          }
        }
      ],
      "inertial": {
        "origin": {
          "xyz": [
            0.000846,
            -1.933e-06,
            4.2e-05
          ],
          "rpy": [
            0.0,
            0.0,
            1.5707963267948966
          ]
        },
        "mass": 0.00434,
        "inertia": {
          "ixx": 5.923e-07,
          "ixy": 0.0,
          "ixz": 3.195e-07,
          "iyy": 1.1156e-06,
          "iyz": -4e-10,
          "izz": 5.743e-07
        }
      }
    },
    {
      "name": "/gripper_bar_link",
      "visuals": [
        {
          "geometry": {
            "type": "mesh",
            "filename": "rx200/RXA-200-M-8-Gripper-Bar.stl",
            "scale": [
              0.001,
              0.001,
              0.001
            ]
          },
          "origin": {
            "xyz": [
              -0.063,
              0.0,
              0.0
            ],
            "rpy": [
              0.0,
              0.0,
              1.5707963267948966
            ]
          },
          "material": {
            "name": "interbotix_black"
          }
        }
      ],
      "collisions": [
        {
          "geometry": {
            "type": "mesh",
            "filename": "rx200/RXA-200-M-8-Gripper-Bar.stl",
            "scale": [
              0.001,
              0.001,
              0.001
            ]
          },
          "origin": {
            "xyz": [
              -0.063,
              0.0,
              0.0
            ],
            "rpy": [
              0.0,
              0.0,
              1.5707963267948966
            ]
          },
          "material": {
            "name": "interbotix_black"
          }
        }
      ],
      "inertial": {
        "origin": {
          "xyz": [
            0.009687,
            5.66e-07,
            0.004962
          ],
          "rpy": [
            0.0,
            0.0,
            1.5707963267948966
          ]
        },
        "mass": 0.034199,
        "inertia": {
          "ixx": 7.4125e-06,
          "ixy": -8e-10,
          "ixz": -6e-10,
          "iyy": 2.843e-05,
          "iyz": -1.3889e-06,
          "izz": 2.86e-05
        }
      }
    },
    {
      "name": "/fingers_link",
      "inertial": {
        "origin": {
          "xyz": [
            0,
            0,
            0
          ],
          "rpy": [
            0,
            0,
            0
          ]
        },
        "mass": 0.001,
        "inertia": {
          "ixx": 0.001,
          "ixy": 0.0,
          "ixz": 0.0,
          "iyy": 0.001,
          "iyz": 0.0,
          "izz": 0.001
        }
      }
    },
    {
      "name": "/left_finger_link",
      "visuals": [
        {
          "geometry": {
            "type": "mesh",
            "filename": "rx200/RXA-200-M-9-Finger.stl",
            "scale": [
              0.001,
              0.001,
              0.001
            ]
          },
          "origin": {
            "xyz": [
              0.0,
              0.005,
              0.0
            ],
            "rpy": [
              3.141592653589793,
              3.141592653589793,
              0.0
            ]
          },
          "material": {
            "name": "interbotix_black"
          }
        }
      ],
      "collisions": [
        {
          "geometry": {
            "type": "mesh",
            "filename": "rx200/RXA-200-M-9-Finger.stl",
            "scale": [
              0.001,
              0.001,
              0.001
            ]
          },
          "origin": {
            "xyz": [
              0.0,
              0.005,
              0.0
            ],
            "rpy": [
              3.141592653589793,
              3.141592653589793,
              0.0
            ]
          },
          "material": {
            "name": "interbotix_black"
          }
        }
      ],
      "inertial": {
        "origin": {
          "xyz": [
            0.013816,
            0.0,
            0.0
          ],
          "rpy": [
            3.141592653589793,
            3.141592653589793,
            1.5707963267948966
          ]
        },
        "mass": 0.016246,
        "inertia": {
          "ixx": 4.731e-06,
          "ixy": -4.56e-07,
          "ixz": 0.0,
          "iyy": 1.5506e-06,
          "iyz": 0.0,
          "izz": 3.7467e-06
        }
      }
    },
    {
      "name": "/right_finger_link",
      "visuals": [
        {
          "geometry": {
            "type": "mesh",
            "filename": "rx200/RXA-200-M-9-Finger.stl",
            "scale": [
              0.001,
              0.001,
              0.001
            ]
          },
          "origin": {
            "xyz": [
              0.0,
              -0.005,
              0.0
            ],
            "rpy": [
              0.0,
              3.141592653589793,
              0.0
            ]
          },
          "material": {
            "name": "interbotix_black"
          }
        }
      ],
      "collisions": [
        {
          "geometry": {
            "type": "mesh",
            "filename": "rx200/RXA-200-M-9-Finger.stl",
            "scale": [
              0.001,
              0.001,
              0.001
            ]
          },
          "origin": {
            "xyz": [
              0.0,
              -0.005,
              0.0
            ],
            "rpy": [
              0.0,
              3.141592653589793,
              0.0
            ]
          },
          "material": {
            "name": "interbotix_black"
          }
        }
      ],
      "inertial": {
        "origin": {
          "xyz": [
            0.013816,
            0.0,
            0.0
          ],
          "rpy": [
            0.0,
            3.141592653589793,
            1.5707963267948966
          ]
        },
        "mass": 0.016246,
        "inertia": {
          "ixx": 4.731e-06,
          "ixy": 4.56e-07,
          "ixz": 0.0,
          "iyy": 1.5506e-06,
          "iyz": 0.0,
          "izz": 3.7467e-06
        }
      }
    },
    {
      "name": "/ee_gripper_link",
      "inertial": {
        "origin": {
          "xyz": [
            0,
            0,
            0
          ],
          "rpy": [
            0,
            0,
            0
          ]
        },
        "mass": 0.001,
        "inertia": {
          "ixx": 0.001,
          "ixy": 0.0,
          "ixz": 0.0,
          "iyy": 0.001,
          "iyz": 0.0,
          "izz": 0.001
        }
      }
    }
  ],
  "joints": [
    {
      "name": "waist",
      "type": "revolute",
      "parent": "/base_link",
      "child": "/shoulder_link",
      "origin": {
        "xyz": [
          0.0,
          0.0,
          0.065
        ],
        "rpy": [
          0.0,
          0.0,
          3.141592653589793
        ]
      },
      "axis": [
        0.0,
        0.0,
        1.0
      ],
      "limits": {
        "effort": 100.0,
        "velocity": 3.141592653589793,
        "lower": -3.141592653589793,
        "upper": 3.141592653589793
      }
    },
    {
      "name": "shoulder",
      "type": "revolute",
      "parent": "/shoulder_link",
      "child": "/upper_arm_link",
      "origin": {
        "xyz": [
          0.0,
          0.0,
          0.03891
        ],
        "rpy": [
          0.0,
          0.0,
          0.0
        ]
      },
      "axis": [
        0.0,
        -1.0,
        0.0
      ],
      "limits": {
        "effort": 100.0,
        "velocity": 1.0,
        "lower": -1.8675022996339325,
        "upper": 1.9373154697137058
      }
    },
    {
      "name": "elbow",
      "type": "revolute",
      "parent": "/upper_arm_link",
      "child": "/forearm_link",
      "origin": {
        "xyz": [
          0.05,
          0.0,
          0.2
        ],
        "rpy": [
          3.141592653589793,
          0.0,
          0.0
        ]
      },
      "axis": [
        0.0,
        1.0,
        0.0
      ],
      "limits": {
        "effort": 100.0,
        "velocity": 3.141592653589793,
        "lower": -1.6231562043547265,
        "upper": 1.8849555921538759
      }
    },
    {
      "name": "wrist_angle",
      "type": "revolute",
      "parent": "/forearm_link",
      "child": "/wrist_link",
      "origin": {
        "xyz": [
          0.2,
          0.0,
          0.0
        ],
        "rpy": [
          0.0,
          0.0,
          0.0
        ]
      },
      "axis": [
        0.0,
        1.0,
        0.0
      ],
      "limits": {
        "effort": 100.0,
        "velocity": 3.141592653589793,
        "lower": -2.1467549799530254,
        "upper": 1.7453292519943295
      }
    },
    {
      "name": "wrist_rotate",
      "type": "revolute",
      "parent": "/wrist_link",
      "child": "/gripper_link",
      "origin": {
        "xyz": [
          0.065,
          0.0,
          0.0
        ],
        "rpy": [
          -3.141592653589793,
          0.0,
          0.0
        ]
      },
      "axis": [
        1.0,
        0.0,
        0.0
      ],
      "limits": {
        "effort": 100.0,
        "velocity": 3.141592653589793,
        "lower": -3.141592653589793,
        "upper": 3.141592653589793
      }
    },
    {
      "name": "ee_arm",
      "type": "fixed",
      "parent": "/gripper_link",
      "child": "/ee_arm_link",
      "origin": {
        "xyz": [
          0.043,
          0.0,
          0.0
        ],
        "rpy": [
          0.0,
          0.0,
          0.0
        ]
      },
      "axis": [
        1.0,
        0.0,
        0.0
      ]
    },
    {
      "name": "gripper",
      "type": "continuous",
      "parent": "/ee_arm_link",
      "child": "/gripper_prop_link",
      "origin": {
        "xyz": [
          0.0055,
          0.0,
          0.0
        ],
        "rpy": [
          0.0,
          0.0,
          0.0
        ]
      },
      "axis": [
        1.0,
        0.0,
        0.0
      ],
      "limits": {
        "effort": 100.0,
        "velocity": 3.141592653589793,
        "lower": 0,
        "upper": 0
      }
    },
    {
      "name": "gripper_bar",
      "type": "fixed",
      "parent": "/ee_arm_link",
      "child": "/gripper_bar_link",
      "origin": {
        "xyz": [
          0.0,
          0.0,
          0.0
        ],
        "rpy": [
          0.0,
          0.0,
          0.0
        ]
      },
      "axis": [
        1.0,
        0.0,
        0.0
      ]
    },
    {
      "name": "ee_bar",
      "type": "fixed",
      "parent": "/gripper_bar_link",
      "child": "/fingers_link",
      "origin": {
        "xyz": [
          0.023,
          0.0,
          0.0
        ],
        "rpy": [
          0.0,
          0.0,
          0.0
        ]
      },
      "axis": [
        1.0,
        0.0,
        0.0
      ]
    },
    {
      "name": "left_finger",
      "type": "prismatic",
      "parent": "/fingers_link",
      "child": "/left_finger_link",
      "origin": {
        "xyz": [
          0.0,
          0.0,
          0.0
        ],
        "rpy": [
          0.0,
          0.0,
          0.0
        ]
      },
      "axis": [
        0.0,
        1.0,
        0.0
      ],
      "limits": {
        "effort": 10.0,
        "velocity": 1.0,
        "lower": 0.015,
        "upper": 0.037
      },
      "mimic": {
        "multiplier": 0.014,
        "offset": 0.014,
        "name": "gripper"
      }
    },
    {
      "name": "right_finger",
      "type": "prismatic",
      "parent": "/fingers_link",
      "child": "/right_finger_link",
      "origin": {
        "xyz": [
          0.0,
          0.0,
          0.0
        ],
        "rpy": [
          0.0,
          0.0,
          0.0
        ]
      },
      "axis": [
        0.0,
        1.0,
        0.0
      ],
      "limits": {
        "effort": 10.0,
        "velocity": 1.0,
        "lower": -0.037,
        "upper": -0.015
      },
      "mimic": {
        "multiplier": -1.0,
        "offset": 0.0,
        "name": "left_finger"
      }
    },
    {
      "name": "ee_gripper",
      "type": "fixed",
      "parent": "/fingers_link",
      "child": "/ee_gripper_link",
      "origin": {
        "xyz": [
          0.027575,
          0.0,
          0.0
        ],
        "rpy": [
          0.0,
          0.0,
          0.0
        ]
      },
      "axis": [
        1.0,
        0.0,
        0.0
      ]
    }
  ],
  "materials": [
    {
      "name": "interbotix_black",
      "filename": "rx200/interbotix_black.png"
    }
  ]
}
)";