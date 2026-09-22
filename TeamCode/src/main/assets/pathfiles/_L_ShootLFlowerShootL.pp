{
  "startPoint": {
    "x": 62.722479721900356,
    "y": 131.46407879490152,
    "locked": false,
    "headingDeg": 90
  },
  "lines": [
    {
      "id": "line-muc6ezs7-d9c5jg",
      "color": "#ffc516",
      "name": "ReadyFlowerPos",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 47.473928157589796,
        "y": 118.14542294322132
      },
      "controlPoints": [
        {
          "x": 67.03070683661646,
          "y": 109.40324449594439
        }
      ],
      "heading": {
        "type": "linear",
        "startDeg": 270,
        "endDeg": 90
      }
    },
    {
      "id": "line-muc6ku7d-tf5a38",
      "color": "#CACCD9",
      "name": "IntoFlower1",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 47.40208574739281,
        "y": 131.9762456546929
      },
      "controlPoints": [],
      "heading": {
        "type": "linear",
        "reverse": true,
        "startDeg": 90,
        "endDeg": 90
      }
    },
    {
      "id": "line-muc6mds6-s7ebyd",
      "color": "#D9A66C",
      "name": "Shot2Left",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 62.900926998841264,
        "y": 131.06431054461183
      },
      "controlPoints": [
        {
          "x": 43.34067207415991,
          "y": 100.95133256083432
        },
        {
          "x": 73.47392815758978,
          "y": 115.37427578215528
        }
      ],
      "heading": {
        "type": "linear",
        "reverse": true,
        "startDeg": 90,
        "endDeg": 270
      }
    },
    {
      "id": "line-muc7iken-y8sh3x",
      "color": "#75B96C",
      "name": "ToPark",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 12.12630359212051,
        "y": 119.82329084588646
      },
      "controlPoints": [
        {
          "x": 72.00521436848202,
          "y": 105.31865585168019
        },
        {
          "x": 19.74623406720741,
          "y": 119.7079953650058
        }
      ],
      "heading": {
        "type": "tangential",
        "reverse": false,
        "startDeg": 270,
        "endDeg": 180
      }
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 141.5,
          "y": 70
        },
        {
          "x": 141.5,
          "y": 141.5
        },
        {
          "x": 118.3,
          "y": 141.5
        },
        {
          "x": 135.5,
          "y": 118
        },
        {
          "x": 136.3,
          "y": 70.2
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6.2,
          "y": 116.9
        },
        {
          "x": 25,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 6,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-muc6ezs7-d9c5jg"
    },
    {
      "kind": "path",
      "lineId": "line-muc6ku7d-tf5a38"
    },
    {
      "kind": "path",
      "lineId": "line-muc6mds6-s7ebyd"
    },
    {
      "kind": "path",
      "lineId": "line-muc7iken-y8sh3x"
    }
  ],
  "fieldPoints": [],
  "version": "1.5.0",
  "timestamp": "2026-09-22T05:04:32.771Z"
}