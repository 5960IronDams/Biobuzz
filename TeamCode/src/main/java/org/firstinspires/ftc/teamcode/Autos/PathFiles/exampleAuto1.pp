{
  "startPoint": {
    "x": 56.34554334554335,
    "y": 9.900488400488394,
    "locked": false,
    "headingDeg": 90
  },
  "lines": [
    {
      "id": "line-mu4x32bi-hjjne5",
      "color": "#ffc516",
      "name": "Path 1",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 35.44017094017094,
        "y": 10.084249084249084
      },
      "controlPoints": [],
      "heading": {
        "type": "linear",
        "startDeg": 90,
        "endDeg": 60
      }
    },
    {
      "id": "line-mu4x3f7v-bt9a3c",
      "color": "#CD9B5C",
      "name": "",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 35.39316239316239,
        "y": 33.843101343101345
      },
      "controlPoints": [],
      "heading": {
        "type": "linear",
        "reverse": true,
        "startDeg": 60,
        "endDeg": 0
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
      "lineId": "line-mu4x32bi-hjjne5"
    },
    {
      "kind": "path",
      "lineId": "line-mu4x3f7v-bt9a3c"
    }
  ],
  "fieldPoints": [],
  "version": "1.5.0",
  "timestamp": "2026-09-17T02:37:04.267Z"
}