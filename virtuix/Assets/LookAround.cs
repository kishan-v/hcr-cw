using UnityEngine;

public class LookAround : MonoBehaviour
{
    public float speed = 3;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update(){
      if(Input.GetMouseButton(0))
      {
        transform.RotateAround(transform.position, -Vector3.up, speed * Input.GetAxis("Mouse X"));
        transform.RotateAround(transform.position, transform.right, speed * Input.GetAxis("Mouse Y"));
      }  
    }
}
