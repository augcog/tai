'use client'

import { useChat, type Message } from 'ai/react'

import { cn } from '@/lib/utils'
import { ChatList } from '@/components/chat-list'
import { ChatPanel } from '@/components/chat-panel'
import { EmptyScreen } from '@/components/empty-screen'
import { ChatScrollAnchor } from '@/components/chat-scroll-anchor'
import { useLocalStorage } from '@/lib/hooks/use-local-storage'
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle
} from '@/components/ui/dialog'
import { useState } from 'react'
import { Button } from './ui/button'
import { Input } from './ui/input'
import { toast } from 'react-hot-toast'
import { usePathname, useRouter } from 'next/navigation'
<<<<<<< HEAD
=======
import { SelectCourse } from '@/components/select-course'
import { useSession } from "next-auth/react"
import { m } from 'framer-motion'
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc

const IS_PREVIEW = process.env.VERCEL_ENV === 'preview'
export interface ChatProps extends React.ComponentProps<'div'> {
  initialMessages?: Message[]
  id?: string
}

export function Chat({ id, initialMessages, className }: ChatProps) {
<<<<<<< HEAD
=======
  const { data: session, status } = useSession()

>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
  const router = useRouter()
  const path = usePathname()
  const [previewToken, setPreviewToken] = useLocalStorage<string | null>(
    'ai-token',
    null
  )
  const [previewTokenDialog, setPreviewTokenDialog] = useState(IS_PREVIEW)
  const [previewTokenInput, setPreviewTokenInput] = useState(previewToken ?? '')
<<<<<<< HEAD
  const { messages, append, reload, stop, isLoading, input, setInput } =
    useChat({
=======

  // Function to save the chat
  const saveChat = async (prev_messages: Message[], input: string, assistant: Message, id:string|undefined) => {

    try {
      // console.log("Saving chat")
      // console.log("prev_messages: ", prev_messages)
      // console.log("input: ", input)
      // console.log("assistant: ", assistant)

      // construct messages
      let messages = prev_messages.map((message) => {
        return {
          role: message.role,
          content: message.content,
        }
      })

      messages.push({
        role: 'user',
        content: input
      })

      messages.push({
        role: 'assistant',
        content: assistant.content
      })

      const response = await fetch('/api/chat/save', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          id,
          messages,
        })
      })

      if (!response.ok) {
        throw new Error('Failed to save chat')
      }

      const data = await response.json()
      console.log('Chat saved successfully:', data)
    } catch (error) {
      console.error('Error saving chat:', error)
    }
  }


  const { messages, append, reload, stop, isLoading, input, setInput } =
    useChat({
      api: '/api/chat',
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
      initialMessages,
      id,
      body: {
        id,
        previewToken
      },
      onResponse(response) {
        if (response.status === 401) {
          toast.error(response.statusText)
        }
      },
<<<<<<< HEAD
      onFinish() {
=======
      onFinish(message: Message) {
        // call /api/chat/save to save the chat
        // console.log("Calling /api/chat/save")
        saveChat(messages, input, message, id);

>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
        if (!path.includes('chat')) {
          window.history.pushState({}, '', `/chat/${id}`)
        }
      }
    })
  return (
    <>
<<<<<<< HEAD
      <div className={cn('pb-[200px] pt-4 md:pt-10', className)}>
=======
      
      <div className={cn('pb-[200px] pt-4 md:pt-10', className)}>
        <div style={{ 'paddingLeft': 10, display: 'inline-block', float: 'left', }}>
          {status === 'authenticated' && (
            <SelectCourse />
          )}

        </div>

>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
        {messages.length ? (
          <>
            <ChatList messages={messages} />
            <ChatScrollAnchor trackVisibility={isLoading} />
          </>
        ) : (
          <EmptyScreen setInput={setInput} />
        )}
      </div>
      <ChatPanel
        id={id}
        isLoading={isLoading}
        stop={stop}
        append={append}
        reload={reload}
        messages={messages}
        input={input}
        setInput={setInput}
      />

      <Dialog open={previewTokenDialog} onOpenChange={setPreviewTokenDialog}>
        <DialogContent>
          <DialogHeader>
            <DialogTitle>Enter your OpenAI Key</DialogTitle>
            <DialogDescription>
              If you have not obtained your OpenAI API key, you can do so by{' '}
              <a
                href="https://platform.openai.com/signup/"
                className="underline"
              >
                signing up
              </a>{' '}
              on the OpenAI website. This is only necessary for preview
              environments so that the open source community can test the app.
              The token will be saved to your browser&apos;s local storage under
              the name <code className="font-mono">ai-token</code>.
            </DialogDescription>
          </DialogHeader>
          <Input
            value={previewTokenInput}
            placeholder="OpenAI API key"
            onChange={e => setPreviewTokenInput(e.target.value)}
          />
          <DialogFooter className="items-center">
            <Button
              onClick={() => {
                setPreviewToken(previewTokenInput)
                setPreviewTokenDialog(false)
              }}
            >
              Save Token
            </Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>
    </>
  )
}
