'use server'

import { revalidatePath } from 'next/cache'
import { redirect } from 'next/navigation'
import { connectToDatabase } from '@/tai/lib/mongodb'
import { auth } from '@/tai/utils/auth'
import { type Chat } from '@/tai/lib/types'

export async function getChats(userId?: string | null) {
  if (!userId) {
    return []
  }

  try {
    const { db } = await connectToDatabase()
    const chatsCollection = db.collection('chats')
    const chats = await chatsCollection
      .find({ userId })
      .sort({ createdAt: -1 })
      .toArray()

    return chats as Chat[]
  } catch (error) {
    console.error('Error retrieving chats:', error)
    return []
  }
}

export async function saveChat(
  title: string,
  messages: any,
  userId: string,
  id: string
) {
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
  const { db } = await connectToDatabase()
  const chats = db.collection('chats')
  await chats.insertOne(payload)
}

export async function getChat(id: string, userId: string) {
  const { db } = await connectToDatabase()
  const chatsCollection = db.collection('chats')
  const chat = await chatsCollection.findOne<Chat>({ id })

  if (!chat || (userId && chat.userId !== userId)) {
    return null
  } else {
    const { _id, ...processedChat } = chat
    return processedChat
  }
}

export async function removeChat({ id, path }: { id: string; path: string }) {
  const session = await auth()

  if (!session) {
    return {
      error: 'Unauthorized'
    }
  }

  const { db } = await connectToDatabase()
  const chatsCollection = db.collection('chats')
  const chat = await chatsCollection.findOne<Chat>({ id })

  if (!chat || chat.userId !== session.user.id) {
    return {
      error: 'Unauthorized'
    }
  }

  await chatsCollection.deleteOne({ id })

  revalidatePath('/')
  return revalidatePath(path)
}

export async function clearChats() {
  const session = await auth()

  if (!session?.user?.id) {
    return {
      error: 'Unauthorized'
    }
  }

  const { db } = await connectToDatabase()
  const chatsCollection = db.collection('chats')
  const result = await chatsCollection.deleteMany({ userId: session.user.id })

  if (result.deletedCount === 0) {
    return redirect('/')
  }

  revalidatePath('/')
  return redirect('/')
}

export async function getSharedChat(id: string) {
  const { db } = await connectToDatabase()
  const chatsCollection = db.collection('chats')
  const chat = await chatsCollection.findOne<Chat>({
    id,
    sharePath: { $exists: true }
  })

  if (!chat) {
    return null
  }

  return chat
}

export async function shareChat(id: string) {
  const session = await auth()

  if (!session?.user?.id) {
    return {
      error: 'Unauthorized'
    }
  }

  const { db } = await connectToDatabase()
  const chatsCollection = db.collection('chats')
  const chat = await chatsCollection.findOne<Chat>({ id })

  if (!chat || chat.userId !== session.user.id) {
    return {
      error: 'Something went wrong'
    }
  }

  const sharePath = `/share/${chat.id}`

  await chatsCollection.updateOne({ id }, { $set: { sharePath } })

  const updatedChat = { ...chat, sharePath }
  return updatedChat
}
