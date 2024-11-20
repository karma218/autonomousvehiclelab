"use client"
import Button from 'react-bootstrap/Button';
import Spinner from 'react-bootstrap/Spinner';
import { useState } from "react";
import Link from 'next/link'
import "./connectPage.css";
import { redirect } from 'next/navigation';


export default function Home() {
  const [isLoading, setLoading] = useState(false);
  
  const connectToAPI = () => {
    setLoading(true);

  }

  const testNav = () => {
    redirect('/control');
  }
  
  return (
  <>
    <div className="connectToScreen">
      <div className="cuteMessage">
        "Welcome to Cal Poly Pomona's Car Controls!!!" 
      </div>

      <Link href="/control"> 
      <Button className="check-if-active" variant="primary" onClick={testNav}>
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
      </Link>
    </div> 
  </>
  );
}

