import { auth } from '@/auth'

export const runtime = 'edge'

export async function POST(req: Request) {
  const json = await req.json();

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

  try {
    var body = JSON.stringify({
      // Adjust payload according to your API requirements
      course: courseId, // Update this as per your API's model naming conventions
      messages,
      temperature: 0.7,
      stream: true,
    });
    
    const apiResponse = await fetch(apiUrl, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${apiKey}`, // Adjust as per your API's auth mechanism
      },
      body: body
    });

    if (apiResponse.ok) {
      return new Response(apiResponse.body, {
        headers: { 'Content-Type': 'application/json' },
      });
    } else {
      return new Response('Error fetching data', { status: apiResponse.status });
    }
  } catch (error) {

    return new Response('Internal Server Error', { status: 500 });
  }
}
