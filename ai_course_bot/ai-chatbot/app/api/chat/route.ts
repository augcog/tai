<<<<<<< HEAD
import { auth } from '@/auth'
<<<<<<< HEAD
=======
import { kv } from '@vercel/kv'
import { nanoid } from '@/lib/utils'
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
=======
import { auth } from '@/tai/utils/auth'
import { kv } from '@vercel/kv'
import { nanoid } from '@/tai/lib/utils'
>>>>>>> def647e43f040b8c085af99271a302bba05d9083

export const runtime = 'edge'

export async function POST(req: Request) {
  const json = await req.json()

<<<<<<< HEAD
  const { messages, previewToken } = json;
  const courseId = messages[0].tool_call_id

  const userId = (await auth())?.user.id;

  if (!userId) {
    return new Response('Unauthorized', {
      status: 401
    });
  }

  // Assuming your API URL and it might require an API key in headers
  const apiUrl: string = process.env['ROAR_BACKEND_HOST'] || "http://0.0.0.0:8000/api/chat/completions";
  const apiKey = previewToken || process.env.YOUR_API_KEY; // Use previewToken if provided, otherwise use your API key
=======
  // console.log("[Route.ts] Request Body: \n", json);
  const { messages, previewToken } = json
  var courseId = messages[messages.length - 1].tool_call_id

  const userId: string = (await auth())?.user.email ?? ''

  if (courseId == null || userId == '') {
    courseId = 'default'
  }

  // Assuming your API URL and it might require an API key in headers
  var apiHost: string =
    process.env['ROAR_BACKEND_HOST'] || 'http://0.0.0.0:9000'

  if (courseId == 'CS 61A') {
    apiHost = process.env['CS_61A_BACKEND_HOST'] || apiHost
  } else if (courseId == 'EE 106b') {
    apiHost = process.env['EE106B_BACKEND_HOST'] || apiHost
  }

<<<<<<< HEAD
  const apiUrl: string = apiHost + '/api/chat/completions';
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
=======
  const apiUrl: string = apiHost + '/api/chat/completions'
>>>>>>> def647e43f040b8c085af99271a302bba05d9083

  try {
    var body = JSON.stringify({
      // Adjust payload according to your API requirements
      course: courseId, // Update this as per your API's model naming conventions
      messages,
      temperature: 0.7,
      stream: true,
<<<<<<< HEAD
    });
    
=======
      userId: userId
    })

    // console.log("[Route.ts] Request Body: \n", body);

>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
    const apiResponse = await fetch(apiUrl, {
      method: 'POST',
      headers: {
<<<<<<< HEAD
        'Content-Type': 'application/json',
<<<<<<< HEAD
        'Authorization': `Bearer ${apiKey}`, // Adjust as per your API's auth mechanism
=======
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
=======
        'Content-Type': 'application/json'
>>>>>>> def647e43f040b8c085af99271a302bba05d9083
      },
      body: body
    })

<<<<<<< HEAD
<<<<<<< HEAD
    if (apiResponse.ok) {
=======
    if (apiResponse.ok) {    
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
=======
    if (apiResponse.ok) {
>>>>>>> def647e43f040b8c085af99271a302bba05d9083
      return new Response(apiResponse.body, {
        headers: { 'Content-Type': 'application/json' }
      })
    } else {
<<<<<<< HEAD
<<<<<<< HEAD
=======
      console.log("[Route.ts] API Response Not Ok");
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
      return new Response('Error fetching data', { status: apiResponse.status });
=======
      console.log('[Route.ts] API Response Not Ok')
      return new Response('Error fetching data', { status: apiResponse.status })
>>>>>>> def647e43f040b8c085af99271a302bba05d9083
    }
  } catch (error) {
    console.log('[Route.ts] Error: ', error)
    return new Response(
      'Internal Server Error; Server may be down, Please try again later',
      { headers: { 'Content-Type': 'application/json' } }
    )
  }
}
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
=======
>>>>>>> def647e43f040b8c085af99271a302bba05d9083
