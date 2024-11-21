"use client"
import Button from 'react-bootstrap/Button';
import Spinner from 'react-bootstrap/Spinner';
import { useState } from "react";
import Link from 'next/link'
import "./connectPage.css";
import { redirect } from 'next/navigation';


export default function Home() {
  const [isAlive, setAlive] = useState(false);
  const [isLoading, setLoading] = useState(false);
  
  const checkCarHeartBeat = async () => {
    setLoading(true);
    try {
      const req = await fetch("http://127.0.0.1:5000/heartbeat"); 
      if (!req.ok){
        throw new Error(`Error: ${req.status}`);
      }

      const data = await req.json(); 

      if (data["is_alive"] !== false) {
        setAlive(true);
        redirect('/control');
      }

      setAlive(false);
    } catch(error) {
      setLoading(false);
    };
  }

  
  return (
  <>
    <div className="connectToScreen">
      <div className="cuteMessage">
        "Welcome to Cal Poly Pomona's Car Controls!!!" 
      </div>

        <Button className="check-if-active" variant="primary" onClick={checkCarHeartBeat}>
          {isLoading == true ? <Spinner
            animation="border" 
            role="status"
            size="sm"
            aria-hidden="true">
          </Spinner> :
            <div></div>
          }
          <span className={isLoading === true ? "visuallyHidden" : "visuallyVisible" }>
            {isLoading === true ? "Connecting..." : "Connect"}
          </span>
        </Button>
    </div> 
  </>
  );
}

