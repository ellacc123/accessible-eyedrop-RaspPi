# firebase_utils.py
import firebase_admin
from firebase_admin import credentials, firestore
import time

# Initialize Firebase and return a firestore client 
def init_firebase():
    cred = credentials.Certificate("serviceAccountKey.json")  #service account key
    firebase_admin.initialize_app(cred) #connect script (main.py to firebase)
    return firestore.client() #returns a firestore client to read/write data

# Create a firestore client instance, so you can call
# db anywhere in script to interact with Firestore
db = init_firebase()

#saves a record of a user action in Firestore
def update_firestore(angle, dropcount, success):
    user_id = "user_123" # hardcoded user
    data = { # data dictionary
        "angle": angle if angle is not None else 0,
        "drops_left_eye": dropcount,
        "success": success,
        "timestamp": int(time.time())
    }

    # creates a new document Users/user_123/administration_records/<new_document>
    try:
        doc_ref = db.collection("Users").document(user_id).collection("administration_records").document()
        doc_ref.set(data)
        print("✅ Firestore updated with:", data)
        return doc_ref.id
    except Exception as e:
        print("❌ Error updating Firestore:", e)
        return None
