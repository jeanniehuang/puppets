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
const IDLE_X_ROTATION = 0.18
const IDLE_Y_ROTATION = -0.2
const IDLE_Z_ROTATION = 0
const HAND_Y_AXIS_TO_ROBOT_Z_OFFSET = -Math.PI / 2
const HAND_YAW_GAIN = 1.65
const HAND_NEAR_SIZE = 0.48
const HAND_FAR_SIZE = 0.04
const ROBOT_NEAR_SCALE = 2.6
const ROBOT_FAR_SCALE = 0.2
const FIST_CLOSURE_OFFSET = 0.3
const ROBOT_BONE_ALIASES = {
  root: 'Bone',
  headUpper: 'Bone.001',
  neck: 'Bone.002',
  leftShoulder: 'Bone.003.L',
  leftElbow: 'Bone.004.L',
  leftWrist: 'Bone.004.L.001',
  leftLeg: 'Bone.005.L',
  rightShoulder: 'Bone.003.R',
  rightElbow: 'Bone.004.R',
  rightWrist: 'Bone.004.R.001',
  rightLeg: 'Bone.005.R',
}

function getExperienceConfig(pathname) {
  const normalizedPath = pathname.replace(/\/+$/, '') || '/'

  if (normalizedPath === '/octopus') {
    return {
      modelUrl: '/robot-octopus.glb',
      modelYawOffset: -Math.PI / 2,
      handYOffset: 0.7,
      statusText: 'Move one hand in front of the camera to steer the octopus.',
    }
  }

  return {
    modelUrl: '/robot-root.glb',
    modelYawOffset: 0,
    handYOffset: 0,
    statusText: 'Move one hand in front of the camera to steer the robot.',
  }
}

function App() {
  const experienceConfigRef = useRef(getExperienceConfig(window.location.pathname))
  const videoRef = useRef(null)
  const overlayCanvasRef = useRef(null)
  const sceneCanvasRef = useRef(null)
  const handLandmarkerRef = useRef(null)
  const streamRef = useRef(null)
  const animationFrameRef = useRef(0)
  const lastVideoTimeRef = useRef(-1)
  const lastRenderTimeRef = useRef(0)
  const viewportRef = useRef({ width: 0, height: 0, dpr: 0 })
  const previousHandTargetRef = useRef(null)
  const handVelocityRef = useRef({ x: 0, y: 0, z: 0 })
  const rigBonesRef = useRef({})
  const bonePhysicsRef = useRef([])
  const animationMixerRef = useRef(null)
  const animationActionsRef = useRef({})
  const previousFistRef = useRef(false)
  const baseRobotScaleRef = useRef(1)
  const rootMotionRef = useRef({
    time: 0,
    position: new THREE.Vector3(),
    velocity: new THREE.Vector3(),
  })
  const handStateRef = useRef({
    visible: false,
    targetX: 0,
    targetY: 0,
    targetZ: 0,
    targetYaw: 0,
    targetPitch: 0,
    targetSpin: 0,
    targetScale: 1,
  })
  const [status, setStatus] = useState('Loading camera, hand tracking, and robot...')

  useEffect(() => {
    const experienceConfig = experienceConfigRef.current
    let cancelled = false
    let renderer = null
    let scene = null
    let camera = null
    let robotRoot = null

    const lerpAngle = (current, target, alpha) => {
      const delta = Math.atan2(Math.sin(target - current), Math.cos(target - current))
      return current + delta * alpha
    }

    const playAnimation = (name) => {
      const actions = animationActionsRef.current
      const nextAction = actions[name]
      const mixer = animationMixerRef.current

      if (!nextAction || !mixer) {
        return
      }

      Object.values(actions).forEach((action) => {
        if (action !== nextAction) {
          action.stop()
        }
      })

      nextAction.reset()
      nextAction.paused = false
      nextAction.enabled = true
      nextAction.setEffectiveTimeScale(1)
      nextAction.setEffectiveWeight(1)
      nextAction.play()
      mixer.setTime(0)
    }

    const updateBonePhysics = () => {
      const entries = bonePhysicsRef.current
      if (!entries.length || !robotRoot) {
        return
      }

      const now = performance.now()
      const motion = rootMotionRef.current
      const dt = motion.time ? Math.max((now - motion.time) / 1000, 1 / 120) : 1 / 60
      const position = robotRoot.position.clone()
      const velocity = position.clone().sub(motion.position).divideScalar(dt)
      const acceleration = velocity.clone().sub(motion.velocity).divideScalar(dt)

      motion.time = now
      motion.position.copy(position)
      motion.velocity.copy(velocity)

      const handVelocity = handVelocityRef.current
      const lateralImpulse = THREE.MathUtils.clamp(
        -velocity.x * 0.03 - acceleration.x * 0.004 + handVelocity.x * 0.018,
        -0.85,
        0.85,
      )
      const verticalImpulse = THREE.MathUtils.clamp(
        velocity.y * 0.03 + acceleration.y * 0.005 + handVelocity.y * 0.02,
        -0.85,
        0.85,
      )
      const depthImpulse = THREE.MathUtils.clamp(
        velocity.z * 0.022 + acceleration.z * 0.003 + handVelocity.z * 0.014,
        -0.6,
        0.6,
      )

      entries.forEach((entry) => {
        entry.angularVelocity.x += (verticalImpulse * entry.verticalInfluence) * dt * 60
        entry.angularVelocity.y += (depthImpulse * entry.depthInfluence * entry.sideSign) * dt * 60
        entry.angularVelocity.z += (lateralImpulse * entry.lateralInfluence * entry.sideSign) * dt * 60

        entry.angularVelocity.x += -entry.offset.x * entry.stiffness * dt * 60
        entry.angularVelocity.y += -entry.offset.y * entry.stiffness * 0.7 * dt * 60
        entry.angularVelocity.z += -entry.offset.z * entry.stiffness * dt * 60

        entry.angularVelocity.multiplyScalar(entry.damping)
        entry.offset.addScaledVector(entry.angularVelocity, dt * 60)

        entry.bone.rotation.x = entry.baseRotation.x + entry.offset.x
        entry.bone.rotation.y = entry.baseRotation.y + entry.offset.y
        entry.bone.rotation.z = entry.baseRotation.z + entry.offset.z
      })
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

    const drawHands = () => {
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
    }

    const updateHandTargets = (results) => {
      const primaryHand = results.landmarks[0]

      if (!primaryHand) {
        handStateRef.current.visible = false
        previousFistRef.current = false
        return
      }

      const wrist = primaryHand[0]
      const thumbTip = primaryHand[4]
      const indexBaseKnuckle = primaryHand[5]
      const indexTip = primaryHand[8]
      const middleBase = primaryHand[9]
      const middleTip = primaryHand[12]
      const ringBase = primaryHand[13]
      const ringTip = primaryHand[16]
      const indexBase = primaryHand[5]
      const pinkyBase = primaryHand[17]
      const pinkyTip = primaryHand[20]

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
      const facingYaw = THREE.MathUtils.clamp(
        Math.atan2(-facingVector.x, facingVector.z) * HAND_YAW_GAIN +
          experienceConfig.modelYawOffset,
        -Math.PI,
        Math.PI,
      )
      const facingPitch = THREE.MathUtils.clamp(
        Math.asin(THREE.MathUtils.clamp(facingVector.y, -1, 1)),
        -0.9,
        0.9,
      )
      const handYAxisAngle = Math.atan2(middle3.y - wrist3.y, middle3.x - wrist3.x)
      const palmWidth = Math.hypot(indexBase.x - pinkyBase.x, indexBase.y - pinkyBase.y)
      const palmLength = Math.hypot(middleBase.x - wrist.x, middleBase.y - wrist.y)
      const handSize = (palmWidth + palmLength) * 0.5

      const targetX = (wrist.x - 0.5) * 7.2
      const targetY = (0.5 - wrist.y) * 4.2 + experienceConfig.handYOffset
      const targetZ = THREE.MathUtils.clamp(wrist.z * 8, -3.5, 1.5)
      const depthProgress = THREE.MathUtils.clamp(
        (handSize - HAND_FAR_SIZE) / (HAND_NEAR_SIZE - HAND_FAR_SIZE),
        0,
        1,
      )
      const targetScale =
        baseRobotScaleRef.current *
        THREE.MathUtils.lerp(ROBOT_FAR_SCALE, ROBOT_NEAR_SCALE, depthProgress)
      const fingerClosures = [
        [thumbTip, indexBaseKnuckle],
        [indexTip, indexBase],
        [middleTip, middleBase],
        [ringTip, ringBase],
        [pinkyTip, pinkyBase],
      ].map(([tip, base]) => {
        const tipDistance = Math.hypot(tip.x - wrist.x, tip.y - wrist.y)
        const baseDistance = Math.hypot(base.x - wrist.x, base.y - wrist.y)
        if (baseDistance <= 0.0001) {
          return 0
        }

        return THREE.MathUtils.clamp(1 - tipDistance / baseDistance, 0, 1)
      })
      const rawFistClosure =
        fingerClosures.reduce((sum, value) => sum + value, 0) / fingerClosures.length
      const fistClosure = THREE.MathUtils.clamp(
        (rawFistClosure - FIST_CLOSURE_OFFSET) / (1 - FIST_CLOSURE_OFFSET),
        0,
        1,
      )
      const isFist = fistClosure >= 0.56

      if (isFist && !previousFistRef.current) {
        playAnimation('Look_Wave')
      }

      previousFistRef.current = isFist
      const now = performance.now()
      const previousTarget = previousHandTargetRef.current

      if (previousTarget) {
        const dt = Math.max((now - previousTarget.time) / 1000, 1 / 120)
        const nextVelocity = {
          x: (targetX - previousTarget.x) / dt,
          y: (targetY - previousTarget.y) / dt,
          z: (targetZ - previousTarget.z) / dt,
        }

        handVelocityRef.current = {
          x: THREE.MathUtils.lerp(handVelocityRef.current.x, nextVelocity.x, 0.22),
          y: THREE.MathUtils.lerp(handVelocityRef.current.y, nextVelocity.y, 0.22),
          z: THREE.MathUtils.lerp(handVelocityRef.current.z, nextVelocity.z, 0.22),
        }
      }

      previousHandTargetRef.current = {
        x: targetX,
        y: targetY,
        z: targetZ,
        time: now,
      }

      handStateRef.current.visible = true
      handStateRef.current.targetX = targetX
      handStateRef.current.targetY = targetY
      handStateRef.current.targetZ = targetZ
      handStateRef.current.targetYaw = facingYaw
      handStateRef.current.targetPitch = facingPitch
      handStateRef.current.targetSpin = handYAxisAngle + HAND_Y_AXIS_TO_ROBOT_Z_OFFSET
      handStateRef.current.targetScale = targetScale
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
      const gltf = await loader.loadAsync(experienceConfig.modelUrl)

      if (cancelled) {
        return
      }

      robotRoot = gltf.scene
      robotRoot.rotation.set(IDLE_X_ROTATION, IDLE_Y_ROTATION, IDLE_Z_ROTATION)

      const mixer = new THREE.AnimationMixer(robotRoot)
      animationMixerRef.current = mixer
      animationActionsRef.current = Object.fromEntries(
        gltf.animations.map((clip) => {
          const action = mixer.clipAction(clip)
          action.clampWhenFinished = true
          action.loop = THREE.LoopOnce
          action.enabled = true
          return [clip.name, action]
        }),
      )

      const bounds = new THREE.Box3().setFromObject(robotRoot)
      const size = bounds.getSize(new THREE.Vector3())
      const center = bounds.getCenter(new THREE.Vector3())
      const maxDimension = Math.max(size.x, size.y, size.z, 0.001)
      const scale = 3 / maxDimension

      baseRobotScaleRef.current = scale
      robotRoot.scale.setScalar(scale)
      robotRoot.position.sub(center.multiplyScalar(scale))
      robotRoot.position.y -= (size.y * scale) * 0.46

      const rigBones = Object.fromEntries(
        Object.entries(ROBOT_BONE_ALIASES).map(([alias, boneName]) => [
          alias,
          robotRoot.getObjectByName(boneName) ?? null,
        ]),
      )
      rigBonesRef.current = rigBones
      rootMotionRef.current = {
        time: performance.now(),
        position: robotRoot.position.clone(),
        velocity: new THREE.Vector3(),
      }

      const rootBone = rigBones.root
      const physicsEntries = []

      robotRoot.traverse((object) => {
        if (!object.isBone || object === rootBone) {
          return
        }

        let depth = 0
        let current = object.parent
        while (current) {
          if (current.isBone) {
            depth += 1
          }
          current = current.parent
        }

        const sideSign = object.name.includes('.L') ? -1 : object.name.includes('.R') ? 1 : 0
        const influenceBase = 0.18 + depth * 0.08
        const isNeck = object.name === ROBOT_BONE_ALIASES.neck

        physicsEntries.push({
          bone: object,
          baseRotation: object.rotation.clone(),
          sideSign,
          stiffness: isNeck
            ? Math.max(0.24, 0.46 - depth * 0.025)
            : Math.max(0.16, 0.34 - depth * 0.035),
          damping: isNeck
            ? Math.min(0.985, 0.93 + depth * 0.02)
            : Math.min(0.96, 0.84 + depth * 0.035),
          lateralInfluence: isNeck
            ? influenceBase * 0.18
            : influenceBase * (sideSign === 0 ? 0.45 : 0.78),
          verticalInfluence: isNeck ? influenceBase * 0.24 : influenceBase * 0.68,
          depthInfluence: isNeck ? influenceBase * 0.16 : influenceBase * 0.5,
          offset: new THREE.Vector3(),
          angularVelocity: new THREE.Vector3(),
        })
      })

      bonePhysicsRef.current = physicsEntries
      scene.add(robotRoot)
      syncViewport()
    }

    const animate = () => {
      if (cancelled || !renderer || !scene || !camera || !robotRoot) {
        return
      }

      const handState = handStateRef.current
      const resized = syncViewport()
      const now = performance.now()
      const deltaSeconds = lastRenderTimeRef.current
        ? Math.min((now - lastRenderTimeRef.current) / 1000, 1 / 20)
        : 1 / 60
      lastRenderTimeRef.current = now

      if (handState.visible) {
        robotRoot.position.x = THREE.MathUtils.lerp(robotRoot.position.x, handState.targetX, 0.28)
        robotRoot.position.y = THREE.MathUtils.lerp(robotRoot.position.y, handState.targetY, 0.28)
        robotRoot.position.z = THREE.MathUtils.lerp(robotRoot.position.z, handState.targetZ, 0.24)
        const nextScale = THREE.MathUtils.lerp(
          robotRoot.scale.x,
          handState.targetScale,
          0.24,
        )
        robotRoot.scale.setScalar(nextScale)
        robotRoot.rotation.x = THREE.MathUtils.lerp(robotRoot.rotation.x, handState.targetPitch, 0.22)
        robotRoot.rotation.y = lerpAngle(robotRoot.rotation.y, handState.targetYaw, 0.4)
        robotRoot.rotation.z = lerpAngle(robotRoot.rotation.z, handState.targetSpin, 0.3)
      } else {
        handVelocityRef.current = {
          x: THREE.MathUtils.lerp(handVelocityRef.current.x, 0, 0.08),
          y: THREE.MathUtils.lerp(handVelocityRef.current.y, 0, 0.08),
          z: THREE.MathUtils.lerp(handVelocityRef.current.z, 0, 0.08),
        }
        robotRoot.position.x = THREE.MathUtils.lerp(robotRoot.position.x, 0, 0.06)
        robotRoot.position.y = THREE.MathUtils.lerp(robotRoot.position.y, -1.15, 0.06)
        robotRoot.position.z = THREE.MathUtils.lerp(robotRoot.position.z, 0, 0.06)
        const nextScale = THREE.MathUtils.lerp(
          robotRoot.scale.x,
          baseRobotScaleRef.current,
          0.12,
        )
        robotRoot.scale.setScalar(nextScale)
        robotRoot.rotation.x = THREE.MathUtils.lerp(robotRoot.rotation.x, IDLE_X_ROTATION, 0.06)
        robotRoot.rotation.y = lerpAngle(
          robotRoot.rotation.y,
          IDLE_Y_ROTATION + experienceConfig.modelYawOffset,
          0.06,
        )
        robotRoot.rotation.z = lerpAngle(robotRoot.rotation.z, IDLE_Z_ROTATION, 0.06)
      }

      updateBonePhysics()
      animationMixerRef.current?.update(deltaSeconds)

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
        setStatus(experienceConfig.statusText)

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
      rigBonesRef.current = {}
      bonePhysicsRef.current = []
      animationActionsRef.current = {}
      animationMixerRef.current?.stopAllAction()
      animationMixerRef.current = null
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
