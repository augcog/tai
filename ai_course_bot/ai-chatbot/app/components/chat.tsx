'use client'

import { useChat, type Message } from 'ai/react'

import { cn } from '@/app/lib/utils'
import { ChatList } from '@/app/components/chat-list'
import { ChatPanel } from '@/app/components/chat-panel'
import { EmptyScreen } from '@/app/components/empty-screen'
import { ChatScrollAnchor } from '@/app/components/chat-scroll-anchor'
import { useLocalStorage } from '@/app/lib/hooks/use-local-storage'
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle
} from '@/app/components/ui/dialog'
import { useState } from 'react'
import { Button } from './ui/button'
import { Input } from './ui/input'
import { toast } from 'react-hot-toast'
import { usePathname, useRouter } from 'next/navigation'
import { SelectCourse } from '@/app/components/select-course'
import { useSession } from 'next-auth/react'
import { m } from 'framer-motion'

const IS_PREVIEW = process.env.VERCEL_ENV === 'preview'
export interface ChatProps extends React.ComponentProps<'div'> {
  initialMessages?: Message[]
  id?: string
}

export function Chat({ id, initialMessages, className }: ChatProps) {
  const { data: session, status } = useSession()

  const router = useRouter()
  const path = usePathname()
  const [previewToken, setPreviewToken] = useLocalStorage<string | null>(
    'ai-token',
    null
  )
  const [previewTokenDialog, setPreviewTokenDialog] = useState(IS_PREVIEW)
  const [previewTokenInput, setPreviewTokenInput] = useState(previewToken ?? '')

  // Function to save the chat
  const saveChat = async (
    prev_messages: Message[],
    input: string,
    assistant: Message,
    id: string | undefined
  ) => {
    try {
      console.log('Saving chat')
      // console.log("prev_messages: ", prev_messages)
      // console.log("input: ", input)
      // console.log("assistant: ", assistant)

      // construct messages
      let messages = prev_messages.map(message => {
        return {
          role: message.role,
          content: message.content
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

      fetch('/api/chat/save', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify({
          id,
          messages
        })
      }).then(res => {
        if (!res.ok) {
          throw new Error('Failed to save chat')
        }
        console.log('Chat saved successfully')
        // const data = await response.json()
        // console.log('Chat saved successfully:', data)
        // router.refresh()

        if (!path.includes('chat')) {
          console.log('Pushing to chat')
          setTimeout(() => {
            window.history.pushState({}, '', `/chat/${id}`)
            window.location.href = `/chat/${id}`
            router.replace(`/chat/${id}`)
          }, 1500)
        }
      })
    } catch (error) {
      console.error('Error saving chat:', error)
    }
  }

  const { messages, append, reload, stop, isLoading, input, setInput } =
    useChat({
      api: '/api/chat',
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
      onFinish(message: Message) {
        // call /api/chat/save to save the chat
        // console.log("Calling /api/chat/save")
        saveChat(messages, input, message, id)
        // if (!path.includes('chat')) {
        //   console.log('Pushing to chat')
        //   window.history.pushState({}, '', `/chat/${id}`)
        // }
      }
    })
  return (
    <>
      <div className={cn('pb-[200px] pt-4 md:pt-10', className)}>
        <div
          style={{ paddingLeft: 10, display: 'inline-block', float: 'left' }}
        >
          {status === 'authenticated' && <SelectCourse />}
        </div>

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
