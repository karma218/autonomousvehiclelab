import Button from 'react-bootstrap/Button';
import Spinner from 'react-bootstrap/Spinner';
import { useState } from "react";
import Link from 'next/link'
import { redirect } from 'next/navigation';


export default function carControls() {
    const sendKeyPressToRos = (event: any) => {
       if (event.key === "w"){

       } else if (event.key === "s") {

       } else if (event.key === "d") {

       } else if (event.key === "a"){

       }
    }

    return (
        <>
            <div> 
                
            </div>
        </>
    );
}
