// Import the functions you need from the SDKs you need
import { initializeApp } from "https://www.gstatic.com/firebasejs/10.5.2/firebase-app.js";
import { getDatabase, ref, set } from "https://www.gstatic.com/firebasejs/10.5.2/firebase-database.js";
// TODO: Add SDKs for Firebase products that you want to use
// https://firebase.google.com/docs/web/setup#available-libraries

// Your web app's Firebase configuration
// For Firebase JS SDK v7.20.0 and later, measurementId is optional
const firebaseConfig = {
  apiKey: "AIzaSyC10boDyhhVdVfeNUidGEmA27k6cHq56po",
  authDomain: "smart-cart-374c8.firebaseapp.com",
  databaseURL: "https://smart-cart-374c8-default-rtdb.europe-west1.firebasedatabase.app",
  projectId: "smart-cart-374c8",
  storageBucket: "smart-cart-374c8.appspot.com",
  messagingSenderId: "608205053337",
  appId: "1:608205053337:web:baad55d7f776e7b7c6c708",
  measurementId: "G-XPC8BC26E9"
};

// Initialize Firebase
const app = initializeApp(firebaseConfig);
const database = getDatabase();

const left = ref(database, "left");
const right = ref(database, "right");

set(left, 10);
set(right, -2);
console.log(123);
console.log(321);

document.getElementById("left").onchange = function(event, value) {
    console.log("left change");
    console.log(event.target.valueAsNumber)
    set(left, event.target.valueAsNumber);
}

document.getElementById("right").onchange = function(event, value) {
    console.log("right change");
    console.log(event.target.valueAsNumber)
    set(right, event.target.valueAsNumber);
}