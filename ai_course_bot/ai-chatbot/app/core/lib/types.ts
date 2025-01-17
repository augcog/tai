import { ObjectId } from 'mongodb'
import { type Message } from 'ai'

export interface Chat {
  _id?: ObjectId
  id: string
  title: string
  createdAt: Date
  userId: string
  path: string
  messages: Message[]
  sharePath?: string
}

export type ServerActionResult<Result> = Promise<
  | Result
  | {
      error: string
    }
>
