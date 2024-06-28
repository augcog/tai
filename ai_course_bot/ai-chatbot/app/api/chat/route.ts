import { auth } from '@/auth'
import { kv } from '@vercel/kv'
import { nanoid } from '@/lib/utils'

export const runtime = 'edge'

export async function POST(req: Request) {
  const json = await req.json();

  // console.log("[Route.ts] Request Body: \n", json);
  const { messages, previewToken } = json;
  var courseId = messages[messages.length - 1].tool_call_id

  const userId: string = (await auth())?.user.email ?? "";
  console.log("Here")
  if (courseId == null || userId == "") {
    courseId = "default"
  } 

  // Assuming your API URL and it might require an API key in headers
  var apiHost:string = process.env['ROAR_BACKEND_HOST'] || "http://0.0.0.0:9000";

  if (courseId == "CS 61A") {
    apiHost = process.env['CS_61A_BACKEND_HOST'] || apiHost
  } else if (courseId == "EE 106b") {
    apiHost = process.env['EE106B_BACKEND_HOST'] || apiHost
  }

  const apiUrl: string = apiHost + '/api/chat/completions';

  try {
    var body = JSON.stringify({
      // Adjust payload according to your API requirements
      course: courseId, // Update this as per your API's model naming conventions
      messages,
      temperature: 0.7,
      stream: true,
      userId: userId
    });

    // console.log("[Route.ts] Request Body: \n", body);

    const apiResponse = await fetch(apiUrl, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: body
    });

    if (apiResponse.ok) {    
      return new Response(apiResponse.body, {
        headers: { 'Content-Type': 'application/json' },
      });
    } else {
      console.log("[Route.ts] API Response Not Ok");
      return new Response('Error fetching data', { status: apiResponse.status });
    }
  } catch (error) {

    return new Response('Internal Server Error', { status: 500 });
  }
}

