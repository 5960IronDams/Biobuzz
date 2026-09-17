{
  "startPoint": {
    "x": 58.13151796060255,
    "y": 9.639629200463489,
    "name": "Robot",
    "locked": false,
    "headingDeg": 90
  },
  "lines": [
    {
      "id": "line-mu544azx-hk2sqe",
      "color": "#ffc516",
      "name": "RedFront",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 57.9675550405562,
        "y": 56.49536500579374
      },
      "controlPoints": [],
      "heading": {
        "type": "linear",
        "startDeg": 90,
        "endDeg": 180
      }
    },
    {
      "id": "line-mu544p5o-41w0rp",
      "color": "#6AAA6B",
      "name": "RedBack",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 58.117033603707995,
        "y": 85.72247972190034
      },
      "controlPoints": [],
      "heading": {
        "type": "tangential",
        "reverse": true
      }
    },
    {
      "id": "line-mu544rmm-dkcxug",
      "color": "#7DB9AC",
      "name": "BlueBack",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 83.9965237543453,
        "y": 85.57647740440324
      },
      "controlPoints": [],
      "heading": {
        "type": "tangential",
        "reverse": true
      }
    },
    {
      "id": "line-mu544u4z-rfd6k8",
      "color": "#C97D77",
      "name": "BlueFront",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 83.73870220162225,
        "y": 56.454229432213204
      },
      "controlPoints": [],
      "heading": {
        "type": "tangential",
        "reverse": true
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
      "lineId": "line-mu544azx-hk2sqe"
    },
    {
      "kind": "path",
      "lineId": "line-mu544p5o-41w0rp"
    },
    {
      "kind": "path",
      "lineId": "line-mu544rmm-dkcxug"
    },
    {
      "kind": "path",
      "lineId": "line-mu544u4z-rfd6k8"
    }
  ],
  "fieldPoints": [],
  "version": "1.5.0",
  "timestamp": "2026-09-17T05:55:21.481Z"
}