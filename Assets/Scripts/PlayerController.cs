using Cinemachine;
using FMOD.Studio;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerController : MonoBehaviour
{
    [Header("References")]
    public Transform aimIKTarget;
    public GameObject[] hoverPoints;
    public Transform orientation;
    public GameObject catObj;
    public Transform catPos;
    public LayerMask lm;
    private Rigidbody rb;
    private Camera cam;
    public CinemachineFreeLook vCam;

    [Header("Movement")]
    public float speed = 10.0f;
    public float frequency = 6f;
    public float damping = 1f;
    public float jumpForce = 10f;
    public float hoverHeight = 1f;
    private float kp;
    private float kd;

    // Misc
    private Vector3 aimIKVel;
    private Vector3 catPosVel;
    private float catRotVel;
    private float catSmoothTime = 0.01f;
    private float smoothIKTime = 0.1f;
    private bool lockedMode = true;

    // Inputs
    private float horizontalInput;
    private float verticalInput;
    private bool jumping;
    private Vector3 moveDir;

    // Debug
    private Vector3[] hitPointDebugs = { Vector3.zero, Vector3.zero, Vector3.zero, Vector3.zero };

    private void Start()
    {
        rb = GetComponent<Rigidbody>();
        cam = Camera.main;
        lockedMode = true;
        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;
    }

    private void Update()
    {
        orientation.transform.localRotation = Quaternion.Euler(0f, cam.transform.localEulerAngles.y, 0f);

        MyInput();
        Aim();
        catObj.transform.position = Vector3.SmoothDamp(catObj.transform.position, catPos.position, ref catPosVel, catSmoothTime);
        catObj.transform.eulerAngles = transform.eulerAngles;

        moveDir = orientation.forward * verticalInput + orientation.right * horizontalInput;
    }

    private void Aim()
    {
        // Aim IK
        Ray ray = cam.ScreenPointToRay(Input.mousePosition);
        RaycastHit hit;
        if (Physics.Raycast(ray, out hit, 100f, lm, QueryTriggerInteraction.Ignore))
            aimIKTarget.position = Vector3.SmoothDamp(aimIKTarget.position, hit.point, ref aimIKVel, smoothIKTime);
        else
            aimIKTarget.position = Vector3.SmoothDamp(aimIKTarget.position, ray.GetPoint(100f), ref aimIKVel, smoothIKTime);

        if (Input.GetKeyDown(KeyCode.Mouse1))
        {
            lockedMode = !lockedMode;
            if (lockedMode)
            {
                Cursor.lockState = CursorLockMode.Locked;
                Cursor.visible = false;
                vCam.m_XAxis.m_InputAxisName = "Mouse X";
                vCam.m_YAxis.m_InputAxisName = "Mouse Y";
            }
            else
            {
                Cursor.lockState = CursorLockMode.None;
                Cursor.visible = true;
                vCam.m_XAxis.m_InputAxisValue = 0;
                vCam.m_YAxis.m_InputAxisValue = 0;
                vCam.m_XAxis.m_InputAxisName = "";
                vCam.m_YAxis.m_InputAxisName = "";
            }
        }
    }


    private void MyInput()
    {
        jumping = false;
        horizontalInput = Input.GetAxis("Horizontal");
        verticalInput = Input.GetAxis("Vertical");
        jumping = Input.GetKeyDown(KeyCode.Space);
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        PIDTorque();
        PIDForce();

        if (jumping)
            rb.AddForce(Vector3.up * jumpForce * rb.mass);

        for (int i = 0; i < hoverPoints.Length; i++)
        {
            //Raycast down from each hover point
            Ray ray = new Ray(hoverPoints[i].transform.position, Vector3.down * hoverHeight);
            RaycastHit hit;
            Debug.DrawRay(ray.origin, ray.direction * hoverHeight, Color.red);
            if (Physics.Raycast(ray, out hit, hoverHeight * 10f, lm, QueryTriggerInteraction.Ignore))
            {
                hitPointDebugs[i] = hit.point;
                //If the raycast hits the ground, apply force to the hover point
                rb.AddForceAtPosition(Vector3.up * 10.0f * rb.mass * (1.0f - hit.distance), hoverPoints[i].transform.position);
            }
            else
            {
                hitPointDebugs[i] = new Vector3(
                    ray.origin.x + ray.direction.x,
                    ray.origin.y + ray.direction.y,
                    ray.origin.z + ray.direction.z
                    );
            }
        }
    }

    public void PIDForce()
    {
        float dt = Time.fixedDeltaTime;
        float g = 1 / (1 + kd * dt + kp * dt * dt);
        float ksg = kp * g;
        float kdg = (kd + kp * dt) * g;
        Vector3 Pt0 = transform.position;
        Vector3 Vt0 = rb.velocity;
        Vector3 F = ((transform.position + (moveDir.normalized * speed)) - Pt0) * ksg + ((moveDir.normalized * speed) - Vt0) * kdg;
        rb.AddForce(F);
    }
    public void PIDTorque()
    {
        Quaternion desiredRotation = transform.rotation;

        if (moveDir != Vector3.zero)
            desiredRotation = Quaternion.LookRotation(moveDir);

        Quaternion.LookRotation(moveDir);
        kp = (6f * frequency) * (6f * frequency) * 0.25f;
        kd = 4.5f * frequency * damping;
        float dt = Time.fixedDeltaTime;
        float g = 1 / (1 + kd * dt + kp * dt * dt);
        float ksg = kp * g;
        float kdg = (kd + kp * dt) * g;
        Vector3 x;
        float xMag;
        Quaternion q = desiredRotation * Quaternion.Inverse(transform.rotation);
        // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
        // We want the equivalant short rotation eg. -10 degrees
        // Check if rotation is greater than 190 degees == q.w is negative
        if (q.w < 0)
        {
            // Convert the quaterion to eqivalent "short way around" quaterion
            q.x = -q.x;
            q.y = -q.y;
            q.z = -q.z;
            q.w = -q.w;
        }
        q.ToAngleAxis(out xMag, out x);
        x.Normalize();
        x *= Mathf.Deg2Rad;
        Vector3 pidv = kp * x * xMag - kd * rb.angularVelocity;
        Quaternion rotInertia2World = rb.inertiaTensorRotation * transform.rotation;
        pidv = Quaternion.Inverse(rotInertia2World) * pidv;
        pidv.Scale(rb.inertiaTensor);
        pidv = rotInertia2World * pidv;
        rb.AddTorque(pidv);
    }

    private void OnDrawGizmos()
    {
        foreach (Vector3 hitPointDebug in hitPointDebugs)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(hitPointDebug, 0.2f);
        }
    }
}
