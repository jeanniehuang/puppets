import { useEffect, useRef, useState } from 'react'
import * as THREE from 'three'
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js'
import { FilesetResolver, HandLandmarker } from '@mediapipe/tasks-vision'
import './App.css'

const HAND_CONNECTIONS = [
  [0, 1],
  [1, 2],
  [2, 3],
  [3, 4],
  [0, 5],
  [5, 6],
  [6, 7],
  [7, 8],
  [5, 9],
  [9, 10],
  [10, 11],
  [11, 12],
  [9, 13],
  [13, 14],
  [14, 15],
  [15, 16],
  [13, 17],
  [0, 17],
  [17, 18],
  [18, 19],
  [19, 20],
]

const WASM_URL = 'https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision/wasm'
const MODEL_URL =
  'https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task'
const ROBOT_URL = '/robot.glb'
const IDLE_X_ROTATION = 0.18
const IDLE_Y_ROTATION = -0.2
const IDLE_Z_ROTATION = 0
const HAND_Y_AXIS_TO_ROBOT_Z_OFFSET = -Math.PI / 2

function App() {
  const videoRef = useRef(null)
  const overlayCanvasRef = useRef(null)
  const sceneCanvasRef = useRef(null)
  const handLandmarkerRef = useRef(null)
  const streamRef = useRef(null)
  const animationFrameRef = useRef(0)
  const lastVideoTimeRef = useRef(-1)
  const viewportRef = useRef({ width: 0, height: 0, dpr: 0 })
  const handStateRef = useRef({
    visible: false,
    targetX: 0,
    targetY: 0,
    targetZ: 0,
    targetYaw: 0,
    targetPitch: 0,
    targetSpin: 0,
  })
  const [status, setStatus] = useState('Loading camera, hand tracking, and robot...')

  useEffect(() => {
    let cancelled = false
    let renderer = null
    let scene = null
    let camera = null
    let robotRoot = null

    const lerpAngle = (current, target, alpha) => {
      const delta = Math.atan2(Math.sin(target - current), Math.cos(target - current))
      return current + delta * alpha
    }

    const syncViewport = () => {
      const video = videoRef.current
      const overlayCanvas = overlayCanvasRef.current
      const sceneCanvas = sceneCanvasRef.current

      if (!video || !overlayCanvas || !sceneCanvas || !camera || !renderer) {
        return false
      }

      const bounds = video.getBoundingClientRect()
      const dpr = window.devicePixelRatio || 1
      const nextWidth = Math.max(1, Math.round(bounds.width * dpr))
      const nextHeight = Math.max(1, Math.round(bounds.height * dpr))
      const current = viewportRef.current

      if (
        current.width === nextWidth &&
        current.height === nextHeight &&
        current.dpr === dpr
      ) {
        return false
      }

      overlayCanvas.width = nextWidth
      overlayCanvas.height = nextHeight
      viewportRef.current = {
        width: nextWidth,
        height: nextHeight,
        dpr,
      }

      const overlayContext = overlayCanvas.getContext('2d')
      if (overlayContext) {
        overlayContext.setTransform(dpr, 0, 0, dpr, 0, 0)
      }

      renderer.setPixelRatio(dpr)
      renderer.setSize(bounds.width, bounds.height, false)
      camera.aspect = bounds.width / bounds.height
      camera.updateProjectionMatrix()

      return true
    }

    const drawHands = (results) => {
      const canvas = overlayCanvasRef.current
      if (!canvas) {
        return
      }

      const context = canvas.getContext('2d')
      if (!context) {
        return
      }

      const width = canvas.clientWidth
      const height = canvas.clientHeight

      context.clearRect(0, 0, width, height)

      results.landmarks.forEach((hand, index) => {
        const label =
          results.handedness[index]?.[0]?.displayName ??
          results.handedness[index]?.[0]?.categoryName

        context.strokeStyle = 'rgba(112, 255, 218, 0.95)'
        context.lineWidth = 3
        context.lineCap = 'round'
        context.lineJoin = 'round'

        HAND_CONNECTIONS.forEach(([from, to]) => {
          const start = hand[from]
          const end = hand[to]

          context.beginPath()
          context.moveTo(start.x * width, start.y * height)
          context.lineTo(end.x * width, end.y * height)
          context.stroke()
        })

        hand.forEach((point) => {
          context.beginPath()
          context.fillStyle = 'rgba(255, 105, 105, 0.95)'
          context.arc(point.x * width, point.y * height, 5, 0, Math.PI * 2)
          context.fill()
        })

        if (label && hand[0]) {
          context.fillStyle = 'rgba(9, 15, 27, 0.8)'
          context.fillRect(hand[0].x * width - 30, hand[0].y * height - 34, 72, 24)
          context.fillStyle = '#ffffff'
          context.font = '12px SFMono-Regular, Consolas, monospace'
          context.fillText(label, hand[0].x * width - 22, hand[0].y * height - 18)
        }
      })
    }

    const updateHandTargets = (results) => {
      const primaryHand = results.landmarks[0]

      if (!primaryHand) {
        handStateRef.current.visible = false
        return
      }

      const wrist = primaryHand[0]
      const middleBase = primaryHand[9]
      const indexBase = primaryHand[5]
      const pinkyBase = primaryHand[17]

      const toVector3 = (point) => new THREE.Vector3(point.x, -point.y, point.z)
      const wrist3 = toVector3(wrist)
      const index3 = toVector3(indexBase)
      const pinky3 = toVector3(pinkyBase)
      const middle3 = toVector3(middleBase)
      const palmSide = new THREE.Vector3().subVectors(index3, pinky3).normalize()
      const palmUp = new THREE.Vector3().subVectors(middle3, wrist3).normalize()
      const palmNormal = new THREE.Vector3().crossVectors(palmSide, palmUp).normalize()

      if (palmNormal.z < 0) {
        palmNormal.multiplyScalar(-1)
      }

      const facingVector = palmNormal
      const facingYaw = Math.atan2(-facingVector.x, facingVector.z)
      const facingPitch = THREE.MathUtils.clamp(
        Math.asin(THREE.MathUtils.clamp(facingVector.y, -1, 1)),
        -0.9,
        0.9,
      )
      const handYAxisAngle = Math.atan2(middle3.y - wrist3.y, middle3.x - wrist3.x)

      handStateRef.current.visible = true
      handStateRef.current.targetX = (wrist.x - 0.5) * 7.2
      handStateRef.current.targetY = (0.5 - wrist.y) * 4.2
      handStateRef.current.targetZ = THREE.MathUtils.clamp(wrist.z * 8, -3.5, 1.5)
      handStateRef.current.targetYaw = facingYaw
      handStateRef.current.targetPitch = facingPitch
      handStateRef.current.targetSpin = handYAxisAngle + HAND_Y_AXIS_TO_ROBOT_Z_OFFSET
    }

    const startThreeScene = async () => {
      const sceneCanvas = sceneCanvasRef.current
      if (!sceneCanvas) {
        return
      }

      renderer = new THREE.WebGLRenderer({
        canvas: sceneCanvas,
        alpha: true,
        antialias: true,
      })
      renderer.outputColorSpace = THREE.SRGBColorSpace
      renderer.setClearColor(0x000000, 0)

      scene = new THREE.Scene()

      camera = new THREE.PerspectiveCamera(34, 1, 0.1, 100)
      camera.position.set(0, 0, 8)

      scene.add(new THREE.AmbientLight(0xffffff, 1.6))

      const keyLight = new THREE.DirectionalLight(0xffffff, 3.2)
      keyLight.position.set(4, 7, 8)
      scene.add(keyLight)

      const rimLight = new THREE.DirectionalLight(0x88ccff, 1.1)
      rimLight.position.set(-5, 2, 6)
      scene.add(rimLight)

      const loader = new GLTFLoader()
      const gltf = await loader.loadAsync(ROBOT_URL)

      if (cancelled) {
        return
      }

      robotRoot = gltf.scene
      robotRoot.rotation.set(IDLE_X_ROTATION, IDLE_Y_ROTATION, IDLE_Z_ROTATION)

      const bounds = new THREE.Box3().setFromObject(robotRoot)
      const size = bounds.getSize(new THREE.Vector3())
      const center = bounds.getCenter(new THREE.Vector3())
      const maxDimension = Math.max(size.x, size.y, size.z, 0.001)
      const scale = 3 / maxDimension

      robotRoot.scale.setScalar(scale)
      robotRoot.position.sub(center.multiplyScalar(scale))
      robotRoot.position.y -= (size.y * scale) * 0.46

      scene.add(robotRoot)
      syncViewport()
    }

    const animate = () => {
      if (cancelled || !renderer || !scene || !camera || !robotRoot) {
        return
      }

      const handState = handStateRef.current
      const resized = syncViewport()

      if (handState.visible) {
        robotRoot.position.x = THREE.MathUtils.lerp(robotRoot.position.x, handState.targetX, 0.12)
        robotRoot.position.y = THREE.MathUtils.lerp(robotRoot.position.y, handState.targetY, 0.12)
        robotRoot.position.z = THREE.MathUtils.lerp(robotRoot.position.z, handState.targetZ, 0.12)
        robotRoot.rotation.x = THREE.MathUtils.lerp(robotRoot.rotation.x, handState.targetPitch, 0.12)
        robotRoot.rotation.y = lerpAngle(robotRoot.rotation.y, handState.targetYaw, 0.12)
        robotRoot.rotation.z = lerpAngle(robotRoot.rotation.z, handState.targetSpin, 0.16)
      } else {
        robotRoot.position.x = THREE.MathUtils.lerp(robotRoot.position.x, 0, 0.06)
        robotRoot.position.y = THREE.MathUtils.lerp(robotRoot.position.y, -1.15, 0.06)
        robotRoot.position.z = THREE.MathUtils.lerp(robotRoot.position.z, 0, 0.06)
        robotRoot.rotation.x = THREE.MathUtils.lerp(robotRoot.rotation.x, IDLE_X_ROTATION, 0.06)
        robotRoot.rotation.y = lerpAngle(robotRoot.rotation.y, IDLE_Y_ROTATION, 0.06)
        robotRoot.rotation.z = lerpAngle(robotRoot.rotation.z, IDLE_Z_ROTATION, 0.06)
      }

      if (resized) {
        renderer.render(scene, camera)
      } else {
        renderer.render(scene, camera)
      }

      animationFrameRef.current = window.requestAnimationFrame(animate)
    }

    const setup = async () => {
      try {
        await startThreeScene()

        const stream = await navigator.mediaDevices.getUserMedia({
          video: {
            facingMode: 'user',
          },
          audio: false,
        })

        if (cancelled) {
          stream.getTracks().forEach((track) => track.stop())
          return
        }

        streamRef.current = stream
        const video = videoRef.current

        if (!video) {
          return
        }

        video.srcObject = stream
        await video.play()

        const vision = await FilesetResolver.forVisionTasks(WASM_URL)
        const handLandmarker = await HandLandmarker.createFromOptions(vision, {
          baseOptions: {
            modelAssetPath: MODEL_URL,
          },
          runningMode: 'VIDEO',
          numHands: 1,
          minHandDetectionConfidence: 0.5,
          minHandPresenceConfidence: 0.5,
          minTrackingConfidence: 0.5,
        })

        if (cancelled) {
          handLandmarker.close()
          return
        }

        handLandmarkerRef.current = handLandmarker
        syncViewport()
        setStatus('Move one hand in front of the camera to steer the robot.')

        const processVideo = () => {
          const activeVideo = videoRef.current
          const activeHandLandmarker = handLandmarkerRef.current

          if (!activeVideo || !activeHandLandmarker || cancelled) {
            return
          }

          if (
            activeVideo.readyState >= HTMLMediaElement.HAVE_CURRENT_DATA &&
            activeVideo.currentTime !== lastVideoTimeRef.current
          ) {
            const results = activeHandLandmarker.detectForVideo(activeVideo, performance.now())
            drawHands(results)
            updateHandTargets(results)
            lastVideoTimeRef.current = activeVideo.currentTime
          }

          window.requestAnimationFrame(processVideo)
        }

        animate()
        processVideo()
      } catch (error) {
        const message =
          error instanceof Error
            ? error.message
            : 'Unable to start camera, hand tracking, or robot scene.'
        setStatus(message)
      }
    }

    setup()
    window.addEventListener('resize', syncViewport)

    return () => {
      cancelled = true
      window.removeEventListener('resize', syncViewport)
      window.cancelAnimationFrame(animationFrameRef.current)
      handLandmarkerRef.current?.close()
      handLandmarkerRef.current = null
      streamRef.current?.getTracks().forEach((track) => track.stop())
      streamRef.current = null
      renderer?.dispose()
    }
  }, [])

  return (
    <main className="app-shell">
      <video ref={videoRef} className="camera-feed" playsInline muted />
      <canvas ref={sceneCanvasRef} className="scene-overlay" />
      <canvas ref={overlayCanvasRef} className="tracking-overlay" />
      <div className="status-chip">{status}</div>
    </main>
  )
}

export default App
