import { nanoid } from '@/tai/lib/utils'
import { kv } from '@vercel/kv'
import { auth } from '@/tai/utils/auth'

export async function POST(req: Request) {
  const json = await req.json()
  // console.log("[POST /api/chat/save] Request JSON: \n", json);
  const userId: string = (await auth())?.user.id ?? nanoid()
  const title = json.messages[0].content.substring(0, 100)
  saveChat(title, json.messages, userId, json.id)
  return new Response()
}

// function to save chat
async function saveChat(
  title: string,
  messages: any,
  userId: string,
  id: string
) {
  // console.log("[POST /api/chat/save] Saving chat");
  // console.log("Title: ", title);
  // console.log("Messages: ", messages);
  // console.log("UserId: ", userId);
  // console.log("Id: ", id);
  const createdAt = Date.now()
  const path = `/chat/${id}`
  const payload = {
    id,
    title,
    userId,
    createdAt,
    path,
    messages
  }
  await kv.hmset(`chat:${id}`, payload)
  await kv.zadd(`user:chat:${userId}`, {
    score: createdAt,
    member: `chat:${id}`
  })

  // console.log("[POST /api/chat/save] Chat saved successfully: ", payload);
}
